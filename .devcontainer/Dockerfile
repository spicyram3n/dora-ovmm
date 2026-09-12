ARG DOCKER_REPO
ARG ROS_DISTRO
ARG IMAGE_SUFFIX
FROM $DOCKER_REPO:$ROS_DISTRO$IMAGE_SUFFIX

RUN set -eux; \
    apt-get update || true;                                           \
    apt-get install -y --no-install-recommends curl gnupg;            \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg || true;        \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg

ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install some system stuff    
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    python3-pip python-is-python3 ssh neovim git wget unzip \
    ros-humble-control-msgs \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-moveit \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-task-constructor-capabilities \
    ros-humble-moveit-task-constructor-msgs \
    ros-humble-moveit-task-constructor-visualization \
    ros-humble-octomap-server \
    ros-humble-octomap-rviz-plugins \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-viewer \
    ros-humble-pinocchio \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy-linux \
    libignition-gazebo6 \
    libignition-gazebo6-dev \
    libignition-gazebo6-plugins \
    && apt-mark hold libignition-gazebo6 libignition-gazebo6-dev libignition-gazebo6-plugins \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash
# Install some python stuff
RUN pip install --no-cache-dir \
    "numpy<1.25" \
    open3d \
    scipy \
    scikit-learn \
    networkx \
    pandas \
    openai \
    graphviz \
    requests \
    flask \
    opencv-python \
    transforms3d \
    pyyaml \
    trimesh \
    octomap-python==1.10.0.0 \
    pycollada \
    eclipse-zenoh \
    "rerun-sdk==0.22.1"

# Source ROS environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "[ -f /home/ws/ros2_ws/install/setup.bash ] && source /home/ws/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:/home/ws" >> /home/$USERNAME/.bashrc

USER $USERNAME
CMD ["/bin/bash"]