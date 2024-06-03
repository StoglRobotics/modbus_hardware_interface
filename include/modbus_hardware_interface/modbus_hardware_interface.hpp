// Copyright (c) 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// limitations under the License.

#ifndef MODBUS_HARDWARE_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
#define MODBUS_HARDWARE_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "modbus_hardware_interface/modbus_client.hpp"
#include "modbus_hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace modbus_hardware_interface
{
// Default values for modbus
static const bool USE_PERSISTENT_CONNECTION_DEFAULT = true;
static const int NUMBER_OF_BITS_TO_READ_DEFAULT = 32;
static const char DATA_TYPE_DEFAULT[] = "float_dcba";
static const char READ_FUNCTION_DEFAULT[] = "register";
static const char WRITE_FUNCTION_DEFAULT[] = "register";
static const double NO_CMD = std::numeric_limits<double>::quiet_NaN();

class ModbusHardwareInterface : public hardware_interface::SystemInterface
{
public:
  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // TODO(destogl): remove this method from here

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // TODO(destogl): remove this method from here
  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // TODO(destogl): remove this method from here
  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // TODO(destogl): remove this method from here
  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  MODBUS_HARDWARE_INTERFACE__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * For a given interface the parameters from the IntefaceInfo like register number, bits to read,
   * ... are parsed and the corresponding ModbusInterfaceConfig is created.
   *
   * \param[in] interface info about Command/StateInteface containing the parameters
   * \param[in] access_function
   *
   * \return For StateInterface a ModbusInterfaceReadConfig is returned, for CommandInterface a
   * ModbusInterfaceWriteConfig
   */
  template <typename T>
  T create_config(
    hardware_interface::InterfaceInfo & interface, const std::string & access_function);

  /**
   * Check if the connection to the modbus server is established. If not try to initialize it.
   *
   * \return returns true if connected or connection to modbus server could be established.
   * Otherwise false is returned
   */
  bool connection_established() const;

  /**
   * If the modbus clients is set to non_persistent (which means we don't want to hold the
   * connection if we don't want to send/receive data) the connection is disconnected after
   * reading/writing
   */
  void disconnect_non_persistent_connection() const;

  std::unordered_map<std::string, ModbusInterfaceReadConfig> state_interface_to_config_;
  std::unordered_map<std::string, ModbusInterfaceWriteConfig> command_interface_to_config_;
  std::unordered_map<std::string, double> state_interface_to_states_;
  std::unordered_map<std::string, double> command_interface_to_commands_;
  std::unique_ptr<ModbusClient> client_;
};

}  // namespace modbus_hardware_interface

#endif  // MODBUS_HARDWARE_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
