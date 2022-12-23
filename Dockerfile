FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-ae21266 AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-ae21266

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-gps_mavlink_relay_ros2-*_amd64.deb /gps_mavlink_relay_ros2.deb

RUN apt update && apt install -y --no-install-recommends ./gps_mavlink_relay_ros2.deb \
	&& rm /gps_mavlink_relay_ros2.deb