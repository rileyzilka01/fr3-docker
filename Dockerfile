# Start with an official ROS 2 base image for the desired distribution
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1003
ARG USER_GID=1003
ARG USERNAME=user

COPY requirements.txt requirements.txt
COPY packages.txt packages.txt

# Install essential packages and ROS development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    $(cat packages.txt) \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Register Intel RealSense public key and repository
RUN sudo mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
        sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list && \
    sudo apt-get update && \
    sudo apt-get install -y \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg && \
    sudo apt-get clean && rm -rf /var/lib/apt/lists/*


RUN pip install --upgrade pip
RUN pip install -r requirements.txt

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source ~/fr3-docker/franka_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
    
USER $USERNAME

# Set workspace under repo folder
WORKDIR /home/$USERNAME/fr3-docker/franka_ws

# Copy repository content
COPY . /home/$USERNAME/fr3-docker

# chown stuff and update package info
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/fr3-docker/franka_ws \
    && sudo apt-get update \
    && rosdep update

# Copy entrypoint
COPY ./scripts/franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

# Set default shell and entrypoint
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]

# Ensure container starts in workspace
WORKDIR /home/$USERNAME/fr3-docker
