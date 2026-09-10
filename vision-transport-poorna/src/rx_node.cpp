#include <exception>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "vision_transport/image_codec.hpp"
#include "vision_transport/payload_envelope.hpp"
#include "vision_transport/stream_config.hpp"
#include "vision_transport/zenoh_session.hpp"

namespace
{

constexpr auto kActiveDemandRefreshPeriod = std::chrono::seconds(1);

std::string demand_key(const std::string & data_key)
{
  return data_key + "/_demand";
}

class VisionTransportRxNode : public rclcpp::Node
{
public:
  VisionTransportRxNode()
  : Node("vision_transport_rx")
  {
    const auto config_path = declare_parameter<std::string>("config_path", "/config/rx.yaml");
    const auto zenoh_config_path =
      declare_parameter<std::string>("zenoh_config_path", "/config/zenoh_rx.json5");
    zenoh_ = std::make_unique<vision_transport::ZenohSession>(zenoh_config_path);
    streams_ = vision_transport::load_rx_stream_config(config_path);
    RCLCPP_INFO(get_logger(), "loaded %zu RX stream(s) from %s", streams_.size(), config_path.c_str());

    for (const auto & stream : streams_) {
      RCLCPP_INFO(
        get_logger(),
        "RX stream '%s': zenoh:%s -> %s %s codec:%s",
        stream.name.c_str(),
        stream.zenoh_key.c_str(),
        stream.output_topic.c_str(),
        vision_transport::to_string(stream.type).c_str(),
        stream.codec.type.empty() ? "<from metadata>" : stream.codec.type.c_str());
    }

    for (const auto & stream : streams_) {
      if (stream.type == vision_transport::StreamType::PointCloud) {
        auto state = std::make_shared<CloudStreamState>();
        state->config = stream;
        state->publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
          stream.output_topic,
          rclcpp::SensorDataQoS().keep_last(stream.queue_depth));

        zenoh_->subscribe(stream.zenoh_key, [this, state](const std::vector<std::uint8_t> & bytes) {
          ++state->received;
          state->last_envelope_bytes = bytes.size();
          try {
            const auto envelope = vision_transport::unpack_envelope(bytes);
            auto cloud = vision_transport::decode_point_cloud(envelope.metadata, envelope.payload);
            state->publisher->publish(cloud);
            ++state->published;
          } catch (const std::exception & error) {
            ++state->errors;
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "stream '%s' point-cloud receive/decode failed: %s",
              state->config.name.c_str(),
              error.what());
          }
        });

        cloud_streams_.push_back(std::move(state));
        continue;
      }

      auto state = std::make_shared<ImageStreamState>();
      state->config = stream;
      state->publisher = create_publisher<sensor_msgs::msg::Image>(
        stream.output_topic,
        rclcpp::SensorDataQoS().keep_last(stream.queue_depth));

      zenoh_->subscribe(stream.zenoh_key, [this, state](const std::vector<std::uint8_t> & bytes) {
        ++state->received;
        state->last_envelope_bytes = bytes.size();
        try {
          const auto envelope = vision_transport::unpack_envelope(bytes);
          auto image = state->config.type == vision_transport::StreamType::Depth ?
            vision_transport::decode_depth_image(envelope.metadata, envelope.payload) :
            vision_transport::decode_image(envelope.metadata, envelope.payload);
          state->publisher->publish(image);
          ++state->published;
        } catch (const std::exception & error) {
          ++state->errors;
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "stream '%s' receive/decode failed: %s",
            state->config.name.c_str(),
            error.what());
        }
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
        publish_demand_updates();
      });
  }

private:
  struct ImageStreamState
  {
    vision_transport::StreamConfig config;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    std::atomic<std::uint64_t> received{0};
    std::atomic<std::uint64_t> published{0};
    std::atomic<std::uint64_t> errors{0};
    std::atomic<std::uint64_t> last_envelope_bytes{0};
    bool last_demand_active{false};
    bool has_sent_demand{false};
    std::chrono::steady_clock::time_point last_demand_publish;
  };

  struct CloudStreamState
  {
    vision_transport::StreamConfig config;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    std::atomic<std::uint64_t> received{0};
    std::atomic<std::uint64_t> published{0};
    std::atomic<std::uint64_t> errors{0};
    std::atomic<std::uint64_t> last_envelope_bytes{0};
    bool last_demand_active{false};
    bool has_sent_demand{false};
    std::chrono::steady_clock::time_point last_demand_publish;
  };

  void publish_demand_updates()
  {
    const auto now = std::chrono::steady_clock::now();
    for (const auto & state : image_streams_) {
      const auto active = state->publisher->get_subscription_count() > 0;
      const auto changed = !state->has_sent_demand || active != state->last_demand_active;
      const auto refresh = active && state->has_sent_demand &&
        now - state->last_demand_publish >= kActiveDemandRefreshPeriod;
      if (changed || refresh) {
        publish_demand(state->config, active, changed);
        state->last_demand_active = active;
        state->has_sent_demand = true;
        state->last_demand_publish = now;
      }
    }

    for (const auto & state : cloud_streams_) {
      const auto active = state->publisher->get_subscription_count() > 0;
      const auto changed = !state->has_sent_demand || active != state->last_demand_active;
      const auto refresh = active && state->has_sent_demand &&
        now - state->last_demand_publish >= kActiveDemandRefreshPeriod;
      if (changed || refresh) {
        publish_demand(state->config, active, changed);
        state->last_demand_active = active;
        state->has_sent_demand = true;
        state->last_demand_publish = now;
      }
    }
  }

  void publish_demand(const vision_transport::StreamConfig & stream, bool active, bool log_change)
  {
    try {
      zenoh_->put_cbor(demand_key(stream.zenoh_key), {static_cast<std::uint8_t>(active ? 1 : 0)});
      if (log_change) {
        RCLCPP_INFO(
          get_logger(),
          "RX stream '%s' demand=%s for output topic %s",
          stream.name.c_str(),
          active ? "true" : "false",
          stream.output_topic.c_str());
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "stream '%s' demand publish failed: %s",
        stream.name.c_str(),
        error.what());
    }
  }

  void log_metrics() const
  {
    for (const auto & state : image_streams_) {
      RCLCPP_INFO(
        get_logger(),
        "RX metrics stream='%s' type=%s codec=%s received=%lu published=%lu errors=%lu last_envelope_bytes=%lu demand_active=%s",
        state->config.name.c_str(),
        vision_transport::to_string(state->config.type).c_str(),
        state->config.codec.type.c_str(),
        state->received.load(),
        state->published.load(),
        state->errors.load(),
        state->last_envelope_bytes.load(),
        state->last_demand_active ? "true" : "false");
    }
    for (const auto & state : cloud_streams_) {
      RCLCPP_INFO(
        get_logger(),
        "RX metrics stream='%s' type=pointcloud codec=%s received=%lu published=%lu errors=%lu last_envelope_bytes=%lu demand_active=%s",
        state->config.name.c_str(),
        state->config.codec.type.c_str(),
        state->received.load(),
        state->published.load(),
        state->errors.load(),
        state->last_envelope_bytes.load(),
        state->last_demand_active ? "true" : "false");
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
    rclcpp::spin(std::make_shared<VisionTransportRxNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_transport_rx"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
