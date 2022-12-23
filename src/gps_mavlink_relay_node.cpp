// Copyright (c) 2022 Technology Innovation Institute & Unikie Oy.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.
#include <functional>
#include <memory>
#include "gps_mavlink_relay_ros2/gps_mavlink_relay.hpp"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"

using std::placeholders::_1;
namespace gps_mavlink_relay_ros2
{

GpsMavlinkRelayer::GpsMavlinkRelayer(const rclcpp::NodeOptions & options)
: Node("gps_mavlink_relayer", options)
{
  declare_parameter(
    "mavlink_connection_url", rclcpp::ParameterValue(
      "udp://:14540"));
  rclcpp::QoS best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  declare_parameter("mavlink_publish_rate", rclcpp::ParameterValue(10.0));
  mavlink_publish_period_ = 1. / get_parameter("mavlink_publish_rate").as_double();
  last_mavlink_publish_time_ = rclcpp::Time();

  subscription_ = this->create_subscription<px4_msgs::msg::SensorGps>(
    "~/sensor_gps_in", best_effort_qos, std::bind(&GpsMavlinkRelayer::topic_callback, this, _1));

  mavlink_connection_url_ = get_parameter("mavlink_connection_url").as_string();
  mavsdk_ = std::make_shared<mavsdk::Mavsdk>();
  mavsdk::ConnectionResult connection_result;
  connection_result = mavsdk_->add_any_connection(mavlink_connection_url_);

  if (connection_result != mavsdk::ConnectionResult::Success) {
    // It will not configure the plugin if the connection failed. Causing error in whole stack
    RCLCPP_ERROR(
      get_logger(), "Connection failed with code: %s",
      toString(connection_result).c_str());
    throw;
  } else {
    RCLCPP_INFO(get_logger(), "MAVSDK connected to device: %s", mavlink_connection_url_.c_str());
  }

  // Maybe use subscribe_to_new_system?
  bool connected = false;
  rclcpp::Rate r(10);
  uint retry_cnt = 0;
  while (rclcpp::ok() && !connected) {
    RCLCPP_INFO(get_logger(), "Systems size: %ld", mavsdk_->systems().size());
    if (mavsdk_->systems().size() < 1) {
      RCLCPP_INFO(
        get_logger(), "Waiting for connection at URL: %s",
        mavlink_connection_url_.c_str());
      r.sleep();
      retry_cnt++;
    }
    if (retry_cnt > 300) {
      RCLCPP_ERROR(get_logger(), "No connection to MAVSDK after 30s");
      throw std::runtime_error("No connection to MAVSDK after 30s");
    }
    for (unsigned i = 0; i < mavsdk_->systems().size(); i++) {
      if (mavsdk_->systems().at(i)->get_system_id() == 1) {
        RCLCPP_INFO(
          get_logger(), "Mavsdk System initialized with ID: %u",
          mavsdk_->systems().at(i)->get_system_id());
        connected = true;
        system_ = mavsdk_->systems().at(i);
        break;
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Target connected");
  telemetry_server_ = std::make_shared<mavsdk::TelemetryServer>(system_);
}

GpsMavlinkRelayer::~GpsMavlinkRelayer()
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  if (telemetry_server_) {
    telemetry_server_.reset();
  }
  if (mavsdk_) {
    mavsdk_.reset();
  }
}

void GpsMavlinkRelayer::topic_callback(const px4_msgs::msg::SensorGps::UniquePtr msg)
{
  auto timestamp = rclcpp::Time(msg->time_utc_usec * 1000ULL);
  if (timestamp < last_mavlink_publish_time_) {
    RCLCPP_WARN(
      get_logger(), "TF timestamp is older than previous one..");
    last_mavlink_publish_time_ = rclcpp::Time();
  }
  if ((timestamp - last_mavlink_publish_time_).seconds() >= mavlink_publish_period_) {
    // RCLCPP_INFO(get_logger(), "SensorGps received, timestamp: %ld", msg->timestamp);
    mavsdk::TelemetryServer::RawGps raw_gps;
    raw_gps.timestamp_us = msg->time_utc_usec;
    raw_gps.latitude_deg = msg->lat / 1e7;
    raw_gps.longitude_deg = msg->lon / 1e7;
    raw_gps.absolute_altitude_m = msg->alt / 1e3;
    raw_gps.hdop = msg->hdop;
    raw_gps.vdop = msg->vdop;
    raw_gps.horizontal_uncertainty_m = msg->eph;
    raw_gps.vertical_uncertainty_m = msg->epv;
    raw_gps.velocity_m_s = msg->vel_m_s;
    raw_gps.cog_deg = msg->cog_rad * 180 / M_PI;
    raw_gps.altitude_ellipsoid_m = msg->alt_ellipsoid / 1e3;
    raw_gps.velocity_uncertainty_m_s = msg->s_variance_m_s;
    raw_gps.heading_uncertainty_deg = msg->c_variance_rad * 180 / M_PI;
    raw_gps.yaw_deg = msg->heading * 180 / M_PI;

    mavsdk::TelemetryServer::GpsInfo gps_info;
    gps_info.num_satellites = msg->satellites_used;
    gps_info.fix_type = mavsdk::TelemetryServer::FixType::Fix3D;

    auto time_start_planning_tree = std::chrono::high_resolution_clock::now();
    telemetry_server_->publish_raw_gps(raw_gps, gps_info);
    RCLCPP_INFO(
      get_logger(), "Mavsdk: Published with time %ld, took %.4f s to publish",
      msg->time_utc_usec,
      std::chrono::duration<float>(
        std::chrono::high_resolution_clock::now() -
        time_start_planning_tree).count());
  }
}

} // namespace gps_mavlink_relay_ros2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gps_mavlink_relay_ros2::GpsMavlinkRelayer)
