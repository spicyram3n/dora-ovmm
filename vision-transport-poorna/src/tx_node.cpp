#include <exception>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "vision_transport/image_codec.hpp"
#include "vision_transport/payload_envelope.hpp"
#include "vision_transport/rate_limiter.hpp"
#include "vision_transport/stream_config.hpp"
#include "vision_transport/zenoh_session.hpp"

namespace
{

std::string demand_key(const std::string & data_key)
{
  return data_key + "/_demand";
}

bool demand_payload_is_active(const std::vector<std::uint8_t> & payload)
{
  return !payload.empty() && payload.front() != 0;
}

class VisionTransportTxNode : public rclcpp::Node
{
public:
  VisionTransportTxNode()
  : Node("vision_transport_tx")
  {
    const auto config_path = declare_parameter<std::string>("config_path", "/config/tx.yaml");
    const auto zenoh_config_path =
      declare_parameter<std::string>("zenoh_config_path", "/config/zenoh_tx.json5");
    zenoh_ = std::make_unique<vision_transport::ZenohSession>(zenoh_config_path);
    streams_ = vision_transport::load_tx_stream_config(config_path);
    RCLCPP_INFO(get_logger(), "loaded %zu TX stream(s) from %s", streams_.size(), config_path.c_str());

    for (const auto & stream : streams_) {
      RCLCPP_INFO(
        get_logger(),
        "TX stream '%s': %s %s -> zenoh:%s codec:%s max_rate_hz:%.3f",
        stream.name.c_str(),
        vision_transport::to_string(stream.type).c_str(),
        stream.input_topic.c_str(),
        stream.zenoh_key.c_str(),
        stream.codec.type.c_str(),
        stream.max_rate_hz);
    }

    for (const auto & stream : streams_) {
      if (stream.type == vision_transport::StreamType::PointCloud) {
        auto state = std::make_shared<CloudStreamState>(stream);
        zenoh_->subscribe(demand_key(stream.zenoh_key), [state](const std::vector<std::uint8_t> & payload) {
          state->demand_active = demand_payload_is_active(payload);
        });
        state->start([this, state](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) {
          process_cloud(state, cloud);
        });
        cloud_streams_.push_back(std::move(state));
        continue;
      }

      auto state = std::make_shared<ImageStreamState>(stream);
      zenoh_->subscribe(demand_key(stream.zenoh_key), [state](const std::vector<std::uint8_t> & payload) {
        state->demand_active = demand_payload_is_active(payload);
      });
      state->start([this, state](const sensor_msgs::msg::Image::ConstSharedPtr & image) {
        process_image(state, image);
      });
      image_streams_.push_back(std::move(state));
    }

    metrics_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      [this] {
        log_metrics();
      });
    demand_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this] {
        reconcile_source_subscriptions();
      });
  }

  ~VisionTransportTxNode() override
  {
    for (const auto & state : image_streams_) {
      state->stop();
    }
    for (const auto & state : cloud_streams_) {
      state->stop();
    }
  }

private:
  struct ImageStreamState
  {
    explicit ImageStreamState(const vision_transport::StreamConfig & stream)
    : config(stream), rate_limiter(stream.max_rate_hz)
    {
    }

    vision_transport::StreamConfig config;
    vision_transport::RateLimiter rate_limiter;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    std::uint64_t sequence{0};
    std::atomic<std::uint64_t> published{0};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> errors{0};
    std::atomic<std::uint64_t> last_payload_bytes{0};
    std::atomic<std::uint64_t> last_envelope_bytes{0};
    std::atomic<bool> demand_active{false};
    std::mutex mutex;
    std::condition_variable condition;
    sensor_msgs::msg::Image::ConstSharedPtr latest;
    bool stopping{false};
    std::thread worker;

    void offer(sensor_msgs::msg::Image::ConstSharedPtr image)
    {
      if (!rate_limiter.should_accept(std::chrono::steady_clock::now())) {
        ++dropped;
        return;
      }

      {
        std::lock_guard<std::mutex> lock(mutex);
        if (latest) {
          ++dropped;
        }
        latest = std::move(image);
      }
      condition.notify_one();
    }

    template<typename Process>
    void start(Process process)
    {
      worker = std::thread([this, process = std::move(process)] {
        while (true) {
          sensor_msgs::msg::Image::ConstSharedPtr image;
          {
            std::unique_lock<std::mutex> lock(mutex);
            condition.wait(lock, [this] { return stopping || latest != nullptr; });
            if (stopping && !latest) {
              return;
            }
            image = std::move(latest);
            latest.reset();
          }
          process(image);
        }
      });
    }

    void stop()
    {
      {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
      }
      condition.notify_one();
      if (worker.joinable()) {
        worker.join();
      }
    }

    void clear_pending()
    {
      std::lock_guard<std::mutex> lock(mutex);
      latest.reset();
    }
  };

  struct CloudStreamState
  {
    explicit CloudStreamState(const vision_transport::StreamConfig & stream)
    : config(stream), rate_limiter(stream.max_rate_hz)
    {
    }

    vision_transport::StreamConfig config;
    vision_transport::RateLimiter rate_limiter;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
    std::uint64_t sequence{0};
    std::atomic<std::uint64_t> published{0};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> errors{0};
    std::atomic<std::uint64_t> last_payload_bytes{0};
    std::atomic<std::uint64_t> last_envelope_bytes{0};
    std::atomic<bool> demand_active{false};
    std::mutex mutex;
    std::condition_variable condition;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest;
    bool stopping{false};
    std::thread worker;

    void offer(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
    {
      if (!rate_limiter.should_accept(std::chrono::steady_clock::now())) {
        ++dropped;
        return;
      }

      {
        std::lock_guard<std::mutex> lock(mutex);
        if (latest) {
          ++dropped;
        }
        latest = std::move(cloud);
      }
      condition.notify_one();
    }

    template<typename Process>
    void start(Process process)
    {
      worker = std::thread([this, process = std::move(process)] {
        while (true) {
          sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
          {
            std::unique_lock<std::mutex> lock(mutex);
            condition.wait(lock, [this] { return stopping || latest != nullptr; });
            if (stopping && !latest) {
              return;
            }
            cloud = std::move(latest);
            latest.reset();
          }
          process(cloud);
        }
      });
    }

    void stop()
    {
      {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
      }
      condition.notify_one();
      if (worker.joinable()) {
        worker.join();
      }
    }

    void clear_pending()
    {
      std::lock_guard<std::mutex> lock(mutex);
      latest.reset();
    }
  };

  void reconcile_source_subscriptions()
  {
    for (const auto & state : image_streams_) {
      if (state->demand_active.load()) {
        if (!state->subscription) {
          state->subscription = create_subscription<sensor_msgs::msg::Image>(
            state->config.input_topic,
            rclcpp::SensorDataQoS().keep_last(state->config.queue_depth),
            [state](const sensor_msgs::msg::Image::ConstSharedPtr image) {
              state->offer(image);
            });
          RCLCPP_INFO(
            get_logger(),
            "TX stream '%s' subscribed to source topic %s because RX has a listener",
            state->config.name.c_str(),
            state->config.input_topic.c_str());
        }
      } else if (state->subscription) {
        state->subscription.reset();
        state->clear_pending();
        RCLCPP_INFO(
          get_logger(),
          "TX stream '%s' unsubscribed from source topic %s because RX has no listener",
          state->config.name.c_str(),
          state->config.input_topic.c_str());
      }
    }

    for (const auto & state : cloud_streams_) {
      if (state->demand_active.load()) {
        if (!state->subscription) {
          state->subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
            state->config.input_topic,
            rclcpp::SensorDataQoS().keep_last(state->config.queue_depth),
            [state](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
              state->offer(cloud);
            });
          RCLCPP_INFO(
            get_logger(),
            "TX stream '%s' subscribed to source topic %s because RX has a listener",
            state->config.name.c_str(),
            state->config.input_topic.c_str());
        }
      } else if (state->subscription) {
        state->subscription.reset();
        state->clear_pending();
        RCLCPP_INFO(
          get_logger(),
          "TX stream '%s' unsubscribed from source topic %s because RX has no listener",
          state->config.name.c_str(),
          state->config.input_topic.c_str());
      }
    }
  }

  void process_image(
    const std::shared_ptr<ImageStreamState> & state,
    const sensor_msgs::msg::Image::ConstSharedPtr & image)
  {
    try {
      auto encoded = state->config.type == vision_transport::StreamType::Depth ?
        vision_transport::encode_depth_image(*image, state->config, ++state->sequence) :
        vision_transport::encode_image(*image, state->config, ++state->sequence);
      const auto envelope = vision_transport::pack_envelope({encoded.metadata, encoded.payload});
      zenoh_->put_cbor(state->config.zenoh_key, envelope);
      state->last_payload_bytes = encoded.payload.size();
      state->last_envelope_bytes = envelope.size();
      ++state->published;
    } catch (const std::exception & error) {
      ++state->errors;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "stream '%s' encode/publish failed: %s",
        state->config.name.c_str(),
        error.what());
    }
  }

  void process_cloud(
    const std::shared_ptr<CloudStreamState> & state,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    try {
      auto encoded = vision_transport::encode_point_cloud(*cloud, state->config, ++state->sequence);
      const auto envelope = vision_transport::pack_envelope({encoded.metadata, encoded.payload});
      zenoh_->put_cbor(state->config.zenoh_key, envelope);
      state->last_payload_bytes = encoded.payload.size();
      state->last_envelope_bytes = envelope.size();
      ++state->published;
    } catch (const std::exception & error) {
      ++state->errors;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "stream '%s' point-cloud encode/publish failed: %s",
        state->config.name.c_str(),
        error.what());
    }
  }

  void log_metrics() const
  {
    for (const auto & state : image_streams_) {
      RCLCPP_INFO(
        get_logger(),
        "TX metrics stream='%s' type=%s codec=%s published=%lu dropped=%lu errors=%lu last_payload_bytes=%lu last_envelope_bytes=%lu demand_active=%s",
        state->config.name.c_str(),
        vision_transport::to_string(state->config.type).c_str(),
        state->config.codec.type.c_str(),
        state->published.load(),
        state->dropped.load(),
        state->errors.load(),
        state->last_payload_bytes.load(),
        state->last_envelope_bytes.load(),
        state->demand_active.load() ? "true" : "false");
    }
    for (const auto & state : cloud_streams_) {
      RCLCPP_INFO(
        get_logger(),
        "TX metrics stream='%s' type=pointcloud codec=%s published=%lu dropped=%lu errors=%lu last_payload_bytes=%lu last_envelope_bytes=%lu demand_active=%s",
        state->config.name.c_str(),
        state->config.codec.type.c_str(),
        state->published.load(),
        state->dropped.load(),
        state->errors.load(),
        state->last_payload_bytes.load(),
        state->last_envelope_bytes.load(),
        state->demand_active.load() ? "true" : "false");
    }
  }

  std::vector<vision_transport::StreamConfig> streams_;
  std::unique_ptr<vision_transport::ZenohSession> zenoh_;
  std::vector<std::shared_ptr<ImageStreamState>> image_streams_;
  std::vector<std::shared_ptr<CloudStreamState>> cloud_streams_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  rclcpp::TimerBase::SharedPtr demand_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VisionTransportTxNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_transport_tx"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
