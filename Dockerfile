# Base image.
FROM osrf/ros2:devel

# Install dependencies.
RUN apt update && apt install -y \
    cmake \
    gcc \
    git-core \
    libhidapi-dev \
    usbrelay \
    openssh-server \
    ros-dev-tools \
    ros-rolling-ros-base

# create sshd run dir
RUN mkdir -p /var/run/sshd

# set password
RUN echo 'root:1' | chpasswd 

# set ssh root permit
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config 
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# expose 22
EXPOSE 22

# start sshd
CMD ["/usr/sbin/sshd","-D"]