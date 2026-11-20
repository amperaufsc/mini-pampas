FROM ros:humble
ARG TARGETARCH

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    python3-colcon-common-extensions \
    ros-humble-turtlesim \
    ros-humble-rqt ros-humble-rqt-common-plugins \
    qt6-wayland \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Build stock sllidar_ros2 in /opt/sllidar_ws (kept out of /mini_pampas)
WORKDIR /opt/sllidar_ws
RUN mkdir -p src \
    && git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2 \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Env not tied to project (part of the container)
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Add foxglove (uses TARGETARCH so it builds on both amd64 and Raspberry Pi arm64)
RUN apt-get update && \
    apt-get install -y --no-install-recommends wget libasound2 && \
    wget --output-document ./foxglove.deb \
        "https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-${TARGETARCH}.deb" && \
    apt-get install -y ./foxglove.deb && \
    rm ./foxglove.deb && \
    rm -rf /var/lib/apt/lists/*

USER root
WORKDIR /mini_pampas

# venv can see system ROS packages (rclpy)
RUN uv venv --system-site-packages /opt/venv

# dependency layer: only rebuilds when these files change
COPY pyproject.toml uv.lock* ./
RUN uv sync --no-install-project

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/sllidar_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /mini_pampas/install/setup.bash ] && source /mini_pampas/install/setup.bash' >> /root/.bashrc \
    && echo "alias ls='ls --color=auto'; alias grep='grep --color=auto'" >> /root/.bashrc \
    && echo 'PS1="\[\e[32m\]\u@\h\[\e[0m\]:\[\e[34m\]\w\[\e[0m\]\$ "' >> /root/.bashrc

CMD ["bash"]