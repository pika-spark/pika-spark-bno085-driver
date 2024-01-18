# Copyright Alexander Entinger, MSc / LXRobotics GmbH
FROM docker.io/arm64v8/ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/colcon_ws/src
WORKDIR /tmp/colcon_ws/src
RUN git clone --recursive https://github.com/pika-spark/pika-spark-bno085-driver && \
    cd pika-spark-bno085-driver && \
    git checkout ros

WORKDIR /tmp/colcon_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY start.sh /
RUN chmod ugo+x /start.sh
ENTRYPOINT ["/start.sh"]
