####################################################
#
#   AutoDRIVE + PPO RL training Dockerfile
#
####################################################

FROM ros:humble

RUN apt update \
    && apt install -y --no-install-recommends \
    sudo \
    wget \
    nano \
    vim \
    curl \
    unzip \
    net-tools \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Bridge / simulator I/O (same pins as icra-2026)
RUN pip3 install --no-cache-dir \
    attrdict \
    numpy==1.22.2 \
    pillow \
    opencv-contrib-python==4.10.0.84 \
    eventlet==0.33.3 \
    Flask==1.1.1 \
    Flask-SocketIO==4.1.0 \
    python-socketio==4.2.0 \
    python-engineio==3.13.0 \
    greenlet==1.1.0 \
    gevent==21.12.0 \
    gevent-websocket==0.10.1 \
    Jinja2==3.0.3 \
    itsdangerous==2.0.1 \
    werkzeug==2.0.3 \
    transforms3d

# PPO stack. CPU torch wheel keeps the image usable on Apple Silicon via linux/amd64.
RUN pip3 install --no-cache-dir \
    gymnasium==0.29.1 \
    tensorboard==2.17.1 \
    tqdm \
    rich \
    && pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir stable-baselines3==2.3.2

RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

RUN apt update --fix-missing \
    && apt install -y xvfb ffmpeg libgdal-dev libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*

# Torch/SB3 pull a new setuptools that calls canonicalize_version(strip_trailing_zero=...).
# Debian's python3-packaging on Humble is too old for that. Re-pin after apt so it wins.
RUN pip3 install --no-cache-dir --upgrade --ignore-installed \
    "packaging>=24.2" \
    "setuptools>=70,<81"

COPY autodrive_devkit/. /home/autodrive_devkit/src/autodrive_devkit
COPY autodrive_race/. /home/autodrive_devkit/src/autodrive_race
COPY triton_model_repo/. /home/autodrive_devkit/src/triton_model_repo

RUN cd /home/autodrive_devkit && colcon build --symlink-install
RUN /bin/bash -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc' \
    && /bin/bash -c 'echo "source /home/autodrive_devkit/install/setup.bash" >> ~/.bashrc' \
    && /bin/bash -c 'echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> ~/.bashrc' \
    && /bin/bash -c 'echo "export AUTODRIVE_RACE_ARTIFACT_DIR=/home/autodrive_devkit/src/autodrive_race" >> ~/.bashrc'

WORKDIR /home/autodrive_devkit
EXPOSE 4567 8765 6006

COPY autodrive_devkit.sh /home
ENTRYPOINT ["/home/autodrive_devkit.sh"]
