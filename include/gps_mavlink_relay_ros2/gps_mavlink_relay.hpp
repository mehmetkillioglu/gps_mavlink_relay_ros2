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
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/telemetry_server/telemetry_server.h"

using std::placeholders::_1;
namespace gps_mavlink_relay_ros2
{

class GpsMavlinkRelayer : public rclcpp::Node
{
public:
  GpsMavlinkRelayer(const rclcpp::NodeOptions & options);
  ~GpsMavlinkRelayer();


  static inline std::string toString(const mavsdk::ConnectionResult & result)
  {
    switch (result) {
      case mavsdk::ConnectionResult::Success:
        return "Success, Connection succeeded.";
      case mavsdk::ConnectionResult::Timeout:
        return "Timeout, Connection timed out.";
      case mavsdk::ConnectionResult::SocketError:
        return "SocketError, Socket error.";
      case mavsdk::ConnectionResult::BindError:
        return "BindError, Bind error.";
      case mavsdk::ConnectionResult::SocketConnectionError:
        return "SocketConnectionError, Socket connection error.";
      case mavsdk::ConnectionResult::ConnectionError:
        return "ConnectionError, Connection error.";
      case mavsdk::ConnectionResult::NotImplemented:
        return "NotImplemented, Connection type not implemented.";
      case mavsdk::ConnectionResult::SystemNotConnected:
        return "SystemNotConnected, No system is connected.";
      case mavsdk::ConnectionResult::SystemBusy:
        return "SystemBusy, System is busy.";
      case mavsdk::ConnectionResult::CommandDenied:
        return "CommandDenied, Command is denied.";
      case mavsdk::ConnectionResult::DestinationIpUnknown:
        return "DestinationIpUnknown, Connection IP is unknown.";
      case mavsdk::ConnectionResult::ConnectionsExhausted:
        return "ConnectionsExhausted, Connections exhausted.";
      case mavsdk::ConnectionResult::ConnectionUrlInvalid:
        return "ConnectionUrlInvalid, URL invalid.";
      case mavsdk::ConnectionResult::BaudrateUnknown:
        return "BaudrateUnknown, Baudrate unknown.";
    }
    return "UNKNOWN";
  }

private:
  void topic_callback(const px4_msgs::msg::SensorGps::UniquePtr msg);
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_;
  std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::TelemetryServer> telemetry_server_;
  std::string mavlink_connection_url_;
  double mavlink_publish_period_;
  rclcpp::Time last_mavlink_publish_time_;
};

} // namespace gps_mavlink_relay_ros2
