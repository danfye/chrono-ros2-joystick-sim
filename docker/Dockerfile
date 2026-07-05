FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV CHRONO_VERSION=release/9.0

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    freeglut3-dev \
    git \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libirrlicht-dev \
    mesa-utils \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-joy \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone --depth 1 --branch ${CHRONO_VERSION} https://github.com/projectchrono/chrono.git \
    && cmake -S chrono -B chrono/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_MODULE_VEHICLE=ON \
      -DENABLE_MODULE_IRRLICHT=ON \
      -DENABLE_MODULE_POSTPROCESS=ON \
    && cmake --build chrono/build --parallel "$(nproc)" \
    && cmake --install chrono/build \
    && rm -rf chrono

WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
