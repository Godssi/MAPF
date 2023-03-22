FROM ros:melodic-ros-core-bionic

RUN cd /etc/apt && \
       sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' sources.list

RUN apt-get update && \
    apt-get install -y \
    ros-melodic-desktop-full

# General process for installing ros-melodic
RUN apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-rosinstall-generator \
    python-wstool \
    python-catkin-tools \
    ros-melodic-catkin && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update

RUN apt-get update && \
    apt-get install -y \
    software-properties-common

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENV ROS_WORKSPACE=/catkin_ws

RUN echo 'alias vb="'"vim ~/.bashrc"'"' >> /root/.bashrc && \
    echo 'alias sb="'"source ~/.bashrc"'"' >> /root/.bashrc && \
    echo 'alias cw="'"cd /catkin_ws"'"' >> /root/.bashrc && \
    echo 'alias cs="'"cd /catkin_ws/src"'"' >> /root/.bashrc && \
    echo 'alias cm="'"cd /catkin_ws && catkin_make"'"' >> /root/.bashrc && \
    echo 'alias rmcm="'"cd /catkin_ws && rm -rf build && rm -rf devel"'"' >> /root/.bashrc && \
    echo 'alias rlrb="'"roslaunch robot_run multi_mobile_robot_odom_with_path.launch"'"' >> /root/.bashrc && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc && \
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> /root/.bashrc && \
    echo 'export ROS_HOSTNAME=localhost' >> /root/.bashrc

RUN mkdir -p $ROS_WORKSPACE/src

COPY ./ROS/src $ROS_WORKSPACE/src

WORKDIR $ROS_WORKSPACE/src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin_init_workspace

WORKDIR $ROS_WORKSPACE
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin_make

CMD [ "/bin/bash" ]