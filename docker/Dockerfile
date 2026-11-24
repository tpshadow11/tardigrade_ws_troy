FROM ros:foxy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    git \
    iputils-ping \
    less \
    libblas-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    liblapack-dev \
    libturbojpeg \
    libzmq3-dev \
    libusb-1.0-0-dev \
    nano \
    nlohmann-json3-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-rosdep \
    python3-vcstool \
    ros-foxy-camera-info-manager \
    ros-foxy-compressed-depth-image-transport \
    ros-foxy-compressed-image-transport \
    ros-foxy-cv-bridge \
    ros-foxy-diagnostic-updater \
    ros-foxy-geographic-msgs \
    ros-foxy-image-geometry \
    ros-foxy-image-transport \
    ros-foxy-joy \
    ros-foxy-nmea-msgs \
    ros-foxy-robot-localization \
    ros-foxy-rosbridge-suite \
    ros-foxy-stereo-msgs \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-tf2-ros \
    ros-foxy-theora-image-transport \
    ros-foxy-xacro \
    udev \
    usbutils \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyserial py_trees==2.1.6

# BehaviorTree.CPP 4.x for tardigrade_bt. ROS 2 Foxy has no binary package
# for v4, so build the pinned release from source into /usr/local. ZeroMQ
# (libzmq3-dev above) enables the Groot2 live view.
ARG BTCPP_VERSION=4.6.2
RUN git clone --depth 1 --branch ${BTCPP_VERSION} \
        https://github.com/BehaviorTree/BehaviorTree.CPP.git /tmp/btcpp \
    && cmake -S /tmp/btcpp -B /tmp/btcpp/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBTCPP_UNIT_TESTS=OFF \
        -DBTCPP_EXAMPLES=OFF \
        -DBTCPP_BUILD_TOOLS=OFF \
        -DBTCPP_SQLITE_LOGGING=OFF \
    && cmake --build /tmp/btcpp/build -j"$(nproc)" \
    && cmake --install /tmp/btcpp/build \
    && ldconfig \
    && rm -rf /tmp/btcpp

WORKDIR /ws

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/ros_bashrc.sh /ros_bashrc.sh
RUN chmod +x /ros_entrypoint.sh \
    && chmod +x /ros_bashrc.sh \
    && (rosdep init || true) \
    && echo "source /ros_bashrc.sh" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]
CMD ["bash"]
