# Base image: Ubuntu 22.04
FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    git \
    vim \
    software-properties-common

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Install ROS 2 Humble prerequisites
RUN apt-get update || true  && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \ 
    && apt-get install -y --no-install-recommends \
    apt-utils\
    dialog\
    lsb-release \
    wget \
    less \
    udev \
    sudo \
    build-essential \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    python3-wheel \
    git \
    jq \
    libopencv-dev \
    libpq-dev \
    zstd \
    usbutils \
    libjpeg-dev \
    libpng-dev \
    libglib2.0-dev \
    # GUI dependencies
    # python3-opengl \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    # libglvnd-dev  \
    # libglu1-mesa \
    # libsm6 \
    # libxi6 \
    # libxrandr2 \
    # libxt6 \
    # qtbase5-dev \
    # libqt5core5a \
    # libqt5gui5 \
    # libqt5widgets5 \
    xdg-user-dirs \
    freeglut3-dev \
    mesa-utils \
    libvulkan-dev \
    libglfw3-dev \
    # Remove apt cache to not clutter image layers
    && echo "alias python=python3" >> ~/.bashrc \
    && echo "alias pip=pip3" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/* 



# Add ROS 2 GPG key and sources list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions

# Source ROS 2 environment and setup entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set up Metavision SDK Prophesee
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
    "opencv-python==4.5.5.64" \
    "sk-video==1.1.10" \
    "fire==0.4.0" \
    "numpy==1.23.4" \
    "h5py==3.7.0" \
    pandas \
    scipy \
    matplotlib \
    "ipywidgets==7.6.5" \
    numba \
    tqdm

# Install metavision SDK and Python dependencies
COPY docker/metavision.list /etc/apt/sources.list.d/metavision.list
RUN apt-get update && apt-get install -y \
    metavision-sdk \
    metavision-sdk-python3.9

# Set HDF5_PLUGIN_PATH environment variable
ENV HDF5_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/hdf5/serial/plugins:$HDF5_PLUGIN_PATH"

# Setup entrypoint
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Expose necessary ports
EXPOSE 11311 8888

# Set default command
CMD ["bash"]

# Clean up APT when done to reduce image size
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*
