FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1

RUN apt upgrade -y && \
    apt update && \
    apt install -y \
        build-essential \
        cmake \
        git \
        libbullet-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        wget \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        clang-format && \
    python3 -m pip install -U \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest

    

ARG user=developer

RUN apt-get update && \
    apt-get install --no-install-recommends -y sudo && \ 
    useradd -ms /bin/bash ${user} && \
    echo "$user:$user" | chpasswd && \
    adduser ${user} sudo && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${user}
WORKDIR /home/${user}

COPY . /home/developer/ros2_ws
WORKDIR /home/developer/ros2_ws

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo ". /home/developer/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD [ "sleep", "infinity" ]