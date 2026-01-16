FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

ARG CODE_DIR=/usr/local/src

# Update ROS gpg key (because gpg keys was expired on June 2025)
ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /tmp/ros.key
RUN rm -rf /etc/apt/sources.list.d/ros1-latest.list && \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key && \
    rm /tmp/ros.key && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $( . /etc/os-release && echo $UBUNTU_CODENAME ) main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt update

# Add User ID and Group ID
ARG UNAME=plevio
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

# Add User into sudoers, can run sudo command without password
RUN apt update && apt install -y sudo
RUN usermod -aG sudo ${UNAME}
RUN echo "${UNAME} ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers.d/${UNAME}

#basic environment
RUN apt install -y \
    ca-certificates \
    build-essential \
    git \
    cmake \
    cmake-curses-gui \
    libace-dev \
    libassimp-dev \
    libglew-dev \
    libglfw3-dev \
    libglm-dev \
    libeigen3-dev \
    clang-format \
    unzip

#my favourites
RUN apt install -y \
    vim \
    gdb \
    libpython3-dev \
    python3-dev

# ROS python library
RUN apt update && \
    apt install -y \
    python3-rosbag python3-rospy \
    python3-catkin-tools

# Ceres dependencies
RUN apt update && \
    apt install -y \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev

# Copy codes
COPY ./ /home/${UNAME}/catkin_ws/src/PL-EVIO

# Change owner of some folders for the development
RUN chown -R $UNAME:$UNAME $CODE_DIR
RUN chown -R $UNAME:$UNAME /home/${UNAME}/catkin_ws

USER $UNAME
WORKDIR /home/${UNAME}

# Submodule init
RUN cd /home/${UNAME}/catkin_ws/src/PL-EVIO && \
    git submodule update --init --recursive

# ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${UNAME}/.bashrc

# Build Ceres solver
RUN cd /home/${UNAME}/catkin_ws/src/PL-EVIO/dependences && \
    unzip -o ceres-solver-1.14.0.zip && \
    cd ceres-solver-1.14.0 && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j3 && sudo make install

# Create catkin_ws
RUN mkdir -p /home/${UNAME}/catkin_ws/src && \
    cd catkin_ws && \
    catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build camera_model -j8 && \
    catkin build evio_estimator feature_tracker pose_graph -j8 && \
    echo "source /home/${UNAME}/catkin_ws/devel/setup.bash" >> /home/${UNAME}/.bashrc