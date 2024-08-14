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

#include "modbus_hardware_interface/modbus_hardware_interface.hpp"
#include "modbus_hardware_interface/modbus_exceptions.hpp"

#include <algorithm>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace modbus_hardware_interface
{
hardware_interface::CallbackReturn ModbusHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(Manual) Check if ip is valid?
  // Get IP of server
  std::string modbus_server_ip = info_.hardware_parameters["modbus_server_ip"];
  if (modbus_server_ip.empty())
  {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "ModbusHardwareInterface server ip for this hardware is empty. Need to set a ip address of "
      "the modbus server.");
    return CallbackReturn::ERROR;
  }

  // Get port of server
  // catch and return with error. Print to provide some additional information for user where it
  // failed since we don't provide default value for the port and crash in case the port is
  // unset/invalid
  int modbus_server_port;
  try
  {
    modbus_server_port = std::stoi(info_.hardware_parameters["modbus_server_port"]);
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "ModbusHardwareInterface: Error while getting port for Hardware [" + info.name + "] " +
        e.what());
    return CallbackReturn::ERROR;
  }
  catch (const std::out_of_range & e)
  {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "ModbusHardwareInterface: Error while getting port for Hardware[" + info.name + "] " +
        e.what());
    return CallbackReturn::ERROR;
  }

  // Get connection type (persistent/not persistent) of the modbus client
  // persistent -> we establish connection once and try to hold it the entire time
  // non persistent -> we establish connection before each read/write and then disconnect
  bool use_persistent_connection;
  std::string use_persistent_connection_str =
    info_.hardware_parameters["use_persistent_connection"];
  if (use_persistent_connection_str.empty())
  {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "use_persistent_connection is neither set to true or false for Hardware[" + info.name +
          "]. Use '"
        << USE_PERSISTENT_CONNECTION_DEFAULT << "' as default.");
    use_persistent_connection = USE_PERSISTENT_CONNECTION_DEFAULT;
  }
  else
  {
    use_persistent_connection = hardware_interface::parse_bool(use_persistent_connection_str);
  }

  client_ =
    std::make_unique<ModbusClient>(modbus_server_ip, modbus_server_port, use_persistent_connection);

  // initialize the configurations for each command and state/interface of each joint
  for (hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // get parameters for each StateInterface
    for (auto & state_interface : joint.state_interfaces)
    {
      std::string state_interface_name = joint.name + "/" + state_interface.name;
      // initialize with no command received
      state_interface_to_states_[state_interface_name] = NO_CMD;
      // Not used with modbus
      if (state_interface.parameters["register"].empty())
      {
        continue;
      }
      // read_function, set to default if not provided
      std::string read_function = state_interface.parameters["read_function"];
      if (read_function.empty())
      {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("ModbusHardwareInterface"),
          "read_function is empty for interface[" + state_interface_name + "]. Use '"
            << READ_FUNCTION_DEFAULT << "' as default.");
        read_function = READ_FUNCTION_DEFAULT;
      }
      state_interface_to_config_.insert(std::make_pair(
        state_interface_name,
        create_config<ModbusInterfaceReadConfig>(state_interface, read_function)));
    }

    // get parameters for each CommandInterface
    for (auto & command_interface : joint.command_interfaces)
    {
      std::string command_interface_name = joint.name + "/" + command_interface.name;

      // set initial to no command
      command_interface_to_commands_[command_interface_name] = NO_CMD;
      // Not used with modbus
      if (command_interface.parameters["register"].empty())
      {
        continue;
      }
      // write_function, set to default if not provided
      std::string write_function = command_interface.parameters["write_function"];
      if (write_function.empty())
      {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("ModbusHardwareInterface"),
          "write_function is empty for interface[" + command_interface_name + "]. Use '"
            << WRITE_FUNCTION_DEFAULT << "' as default.");
        write_function = WRITE_FUNCTION_DEFAULT;
      }
      command_interface_to_config_.insert(std::make_pair(
        command_interface_name,
        create_config<ModbusInterfaceWriteConfig>(command_interface, write_function)));
    }
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ModbusHardwareInterface"), "Finished `on_init`!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (client_->is_persistent_connection())
  {
    if (!client_->connect())
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("ModbusHardwareInterface"),
        "could not establish connection to modbus server.");
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ModbusHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & joint : info_.joints)
  {
    for (const auto & state_interface : joint.state_interfaces)
    {
      std::string state_interface_name = joint.name + "/" + state_interface.name;
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, state_interface.name, &state_interface_to_states_[state_interface_name]));
    }
  }

  // sensors
  for (const auto & sensor : info_.sensors)
  {
    for (const auto & state_interface : sensor.state_interfaces)
    {
      std::string state_interface_name = sensor.name + "/" + state_interface.name;
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, state_interface.name, &state_interface_to_states_[state_interface_name]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ModbusHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & joint : info_.joints)
  {
    for (const auto & command_interface : joint.command_interfaces)
    {
      std::string command_interface_name = joint.name + "/" + command_interface.name;
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, command_interface.name,
        &command_interface_to_commands_[command_interface_name]));
    }
  }

  return command_interfaces;
}

// TODO(destogl): remove this method from here
hardware_interface::CallbackReturn ModbusHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ModbusHardwareInterface"), "activate");
  return CallbackReturn::SUCCESS;
}

// TODO(destogl): remove this method from here
hardware_interface::CallbackReturn ModbusHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ModbusHardwareInterface"), "deactivate");
  return CallbackReturn::SUCCESS;
}

// TODO(destogl): remove this method from here
hardware_interface::return_type ModbusHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

// TODO(destogl): remove this method from here
hardware_interface::return_type ModbusHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModbusHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connection_established())
  {
    return hardware_interface::return_type::ERROR;
  }
  for (auto & [name, config] : state_interface_to_config_)
  {
    try
    {
      state_interface_to_states_[name] = client_->read(config);
    }
    catch (const ModbusReadException & e)
    {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("ModbusHardwareInterface"),
        "Failed to read interface<" << name.c_str() << "> with error:" << e.what()
                                    << ". Deactivating.");
      return hardware_interface::return_type::DEACTIVATE;
    }
  }

  disconnect_non_persistent_connection();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModbusHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connection_established())
  {
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & [name, config] : command_interface_to_config_)
  {
    if (config.write_this_interface())
    {
      try
      {
        client_->write(config, static_cast<float>(command_interface_to_commands_.at(name)));
      }
      catch (const ModbusWriteException & e)
      {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("ModbusHardwareInterface"),
          "Failed to write interface<" << name.c_str() << "> with error:" << e.what()
                                       << ". Deactivating.");
        return hardware_interface::return_type::DEACTIVATE;
      }
    }
  }

  disconnect_non_persistent_connection();
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn ModbusHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  client_->disconnect();
  if (client_->connected())
  {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"), "could not close connection to modbus.");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  CallbackReturn ret = CallbackReturn::SUCCESS;

  client_->disconnect();
  if (client_->connected())
  {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "ON_ERROR: could not close connection to modbus.");
    ret = CallbackReturn::ERROR;
  }
  return ret;
}

template <typename T>
T ModbusHardwareInterface::create_config(
  hardware_interface::InterfaceInfo & interface, const std::string & access_function)
{
  // get register
  int reg;
  try
  {
    reg = std::stoi(interface.parameters["register"]);
  }
  catch (const std::invalid_argument & e)
  {
    const auto reg_str = interface.parameters["register"];
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "Could not convert register number  [" << reg_str << "] to int.");
    throw;
  }

  int bits_to_read = NUMBER_OF_BITS_TO_READ_DEFAULT;
  std::string bits_to_read_str = interface.parameters["bits_to_read"];
  if (bits_to_read_str.empty())
  {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "bits_to_read is not set for interface[" + interface.name + "]. Use '"
        << NUMBER_OF_BITS_TO_READ_DEFAULT << "' as default.");
  }
  else
  {
    try
    {
      bits_to_read = std::stoi(bits_to_read_str);
    }
    catch (const std::invalid_argument & e)
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("ModbusHardwareInterface"),
        "Could not convert given bits to read string:[" << bits_to_read_str << "] to an int.");
      throw;
    }
  }

  // conversion_fn, set to default if not provided
  std::string conversion_fn = interface.parameters["conversion_fn"];
  if (conversion_fn.empty())
  {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ModbusHardwareInterface"),
      "conversion_fn is empty for interface[" + interface.name + "]. Use '" << CONVERSION_FN_DEFAULT
                                                                            << "' as default.");
    conversion_fn = CONVERSION_FN_DEFAULT;
  }

  std::string offset_str = interface.parameters["offset"];
  double offset = 0.0;
  if (!offset_str.empty())
  {
    offset = std::stod(offset_str);
  }

  std::string factor_str = interface.parameters["factor"];
  double factor = 1.0;
  if (!factor_str.empty())
  {
    factor = std::stod(factor_str);
  }

  return T(reg, bits_to_read, access_function, conversion_fn, offset, factor);
}

bool ModbusHardwareInterface::connection_established() const
{
  if (client_->is_persistent_connection())
  {
    return client_->connected();
  }
  return client_->connect();
}

void ModbusHardwareInterface::disconnect_non_persistent_connection() const
{
  if (!client_->is_persistent_connection())
  {
    client_->disconnect();
  }
}

}  // namespace modbus_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  modbus_hardware_interface::ModbusHardwareInterface, hardware_interface::SystemInterface)
