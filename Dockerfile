FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-ae21266 AS builder

COPY . /main_ws/src/
RUN apt update && apt install -y --no-install-recommends wget
RUN wget https://github.com/mavlink/MAVSDK/releases/download/v1.0.6/mavsdk_1.0.6_ubuntu20.04_amd64.deb
RUN dpkg -i ./mavsdk_1.0.6_ubuntu20.04_amd64.deb

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-ae21266

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-gps-mavlink-relay-ros2*_amd64.deb /gps_mavlink_relay_ros2.deb
COPY --from=builder /main_ws/src/mavsdk_1.0.6_ubuntu20.04_amd64.deb /mavsdk_1.0.6_ubuntu20.04_amd64.deb
RUN dpkg -i ./mavsdk_1.0.6_ubuntu20.04_amd64.deb

RUN apt install -y --no-install-recommends ./gps_mavlink_relay_ros2.deb \
	&& rm /gps_mavlink_relay_ros2.deb
