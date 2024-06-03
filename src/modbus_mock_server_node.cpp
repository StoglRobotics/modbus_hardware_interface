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

#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

// needs to be here because defines of TRUE FALSE in modbus.h and mimic joints
#include <modbus/modbus.h>  // NOLINT

#define DEFAULT_REGISTER_SIZE 2

struct RegisterWrapper
{
  explicit RegisterWrapper(const int & reg, const int & reg_size)
  : register_(reg), register_size_(reg_size)
  {
  }
  int register_;
  int register_size_ = DEFAULT_REGISTER_SIZE;
};

class ModbusServerNode : public rclcpp::Node
{
public:
  ModbusServerNode() : Node("modbus_server_node"), run_server_(true)
  {
    this->declare_parameter<int>("modbus_server_retry_period_ms", 1000);
    this->declare_parameter<bool>("log_connection_reset", false);
    this->declare_parameter<std::string>("robot_description", "");

    modbus_server_retry_period_ms_ =
      static_cast<int>(this->get_parameter("modbus_server_retry_period_ms").as_int());
    log_connection_reset_ = this->get_parameter("log_connection_reset").as_bool();
    std::string robot_description = this->get_parameter("robot_description").as_string();
    if (robot_description.empty())
    {
      throw std::runtime_error(
        std::string(this->get_name()) +
        ": no robot_description file provided, cannot configure server.");
    }

    // Attempt initial setup
    // for now we are just interested in the first since there is only one.
    // TODO(Manual) make generic
    hardware_interface::HardwareInfo info =
      hardware_interface::parse_control_resources_from_urdf(robot_description)[0];
    config_server_based_on_urdf(info);
    retryModbusSetup(ip_address_, port_);

    // Start the server in a new thread
    std::thread server_thread(&ModbusServerNode::runServer, this);
    server_thread.detach();
  }

  void config_server_based_on_urdf(hardware_interface::HardwareInfo & info)
  {
    ip_address_ = info.hardware_parameters["modbus_server_ip"];

    if (ip_address_.empty())
    {
      throw std::runtime_error(
        "ModbusMockServer: modbus_server_ip is empty for Hardware [" + info.name +
        "]. Need to specify the ip of the server ");
    }

    // Get port of server for each joint
    // catch and rethrow to provide some additional information for user where it failed since we
    // don't provide default value for the port and crash in case the port is unset/invalid
    try
    {
      port_ = std::stoi(info.hardware_parameters["modbus_server_port"]);
    }
    catch (const std::invalid_argument & e)
    {
      throw std::invalid_argument(
        "ModbusMockServer: Error while getting port for Hardware[" + info.name + "] " + e.what());
    }
    catch (const std::out_of_range & e)
    {
      throw std::out_of_range(
        "ModbusMockServer: Error while getting port for Hardware[" + info.name + "] " + e.what());
    }

    uint8_t highest_register = std::numeric_limits<uint8_t>::min();
    for (hardware_interface::ComponentInfo & joint : info.joints)
    {
      for (auto & state_interface : joint.state_interfaces)
      {
        std::string reg_str = state_interface.parameters["register"];
        // not used with modbus
        if (reg_str.empty())
        {
          continue;
        }

        int reg = std::stoi(reg_str);

        if (reg > highest_register)
        {
          highest_register = static_cast<uint8_t>(reg);
        }
      }

      for (auto & command_interface : joint.command_interfaces)
      {
        std::string reg_str = command_interface.parameters["register"];
        // not used with modbus
        if (reg_str.empty())
        {
          continue;
        }
        int reg = std::stoi(reg_str);
        if (reg > highest_register)
        {
          highest_register = static_cast<uint8_t>(reg);
        }
      }
    }
    setupModbusServer(highest_register);
  }

  bool initializeModbusContext(const std::string & ip_address, int port)
  {
    try
    {
      ctx_ = modbus_new_tcp(ip_address.c_str(), port);
      if (ctx_ == NULL)
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to create the libmodbus context");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Modbus TCP context created");

      server_socket_ = modbus_tcp_listen(ctx_, 1);
      if (server_socket_ == -1)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to listen on port %d", port);
        modbus_free(ctx_);
        ctx_ = NULL;
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Listening on port %d", port);
      return true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in initializeModbusContext: %s", e.what());
      cleanupModbus();
      return false;
    }
  }

  bool setupModbusServer(uint8_t highest_register)
  {
    try
    {
      if (highest_register < std::numeric_limits<uint8_t>::max() - 1)
      {
        // Add one since if we get 12 as register we want to have 12 available and not only 11
        highest_register = highest_register + 2;
      }
      RCLCPP_ERROR(this->get_logger(), "Highest register:%i", highest_register);
      // TODO(Manual) add this as parameter or yaml i don't know
      mb_mapping_ =
        modbus_mapping_new(highest_register, highest_register, highest_register, highest_register);
      if (mb_mapping_ == NULL)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to create Modbus mapping");
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = NULL;
        return false;
      }
      for (uint8_t i = 0; i < highest_register; ++i)
      {
        mb_mapping_->tab_bits[i] = i;
        mb_mapping_->tab_input_bits[i] = i;
        mb_mapping_->tab_input_registers[i] = i;
        mb_mapping_->tab_registers[i] = i;
      }
      RCLCPP_INFO(this->get_logger(), "Modbus mapping created");
      return true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in setupModbusServer: %s", e.what());
      cleanupModbus();
      return false;
    }
  }

  bool retryModbusSetup(
    const std::string & ip_address, int port, int max_retries = 3, int retry_delay_ms = 1000)
  {
    int attempts = 0;
    while (attempts < max_retries)
    {
      if (initializeModbusContext(ip_address, port))
      {
        return true;  // Successful setup
      }
      attempts++;
      RCLCPP_WARN(
        this->get_logger(), "Retrying setup (attempt %d of %d) in %d milliseconds", attempts,
        max_retries, retry_delay_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
    }
    RCLCPP_ERROR(
      this->get_logger(), "Failed to set up Modbus server after %d attempts", max_retries);
    return false;  // Return false if all attempts fail
  }

  bool isServerReady() { return ctx_ != nullptr && mb_mapping_ != nullptr; }

  void runServer()
  {
    RCLCPP_INFO(this->get_logger(), "Modbus server is ready to accept connections");
    // Main server loop for accepting clients
    while (run_server_ && rclcpp::ok())
    {
      if (!isServerReady())
      {
        RCLCPP_WARN(this->get_logger(), "Server not ready to handle requests");
        retryModbusSetup(ip_address_, port_, 1, modbus_server_retry_period_ms_);
        continue;
      }

      int client_socket = modbus_tcp_accept(ctx_, &server_socket_);
      if (client_socket == -1)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          // No client connection attempt, continue
          continue;
        }
        else
        {
          // Log error and break out of the loop
          RCLCPP_ERROR(
            this->get_logger(), "Error accepting client connection: %s", modbus_strerror(errno));
          cleanupModbus();  // Clean up resources
          retryModbusSetup(ip_address_, port_);
        }
      }

      // Create a new thread for each client connection
      std::thread client_thread = std::thread(&ModbusServerNode::handleClient, this, client_socket);
      client_thread.detach();
    }
  }

  void handleClient(int client_socket)
  {
    // Create a new Modbus context for each client
    modbus_t * client_ctx = modbus_new_tcp(ip_address_.c_str(), port_);
    if (!client_ctx)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create context for client");
      close(client_socket);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Got new connection!");
    // Set the new file descriptor for the client context
    modbus_set_socket(client_ctx, client_socket);

    while (run_server_ && rclcpp::ok())
    {
      uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
      int rc = modbus_receive(client_ctx, query);

      // Convert the query array elements to integers and print them
      std::string query_str;
      for (int i = 0; i < MODBUS_TCP_MAX_ADU_LENGTH; ++i)
      {
        query_str += std::to_string(query[i]) + ", ";
      }

      // Print the value of rc and the content of the query array
      RCLCPP_INFO(
        this->get_logger(), "New query return is %i with query %s", rc, query_str.c_str());
      if (rc == -1)
      {
        // Log error and break out of the loop
        if (errno == ECONNRESET)
        {
          if (log_connection_reset_)
          {
            RCLCPP_WARN(this->get_logger(), "Connection reset by peer, client disconnected");
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", modbus_strerror(errno));
        }
        break;
      }

      // Lock resources while processing the request
      std::lock_guard<std::mutex> lock(resource_mutex_);
      modbus_reply(client_ctx, query, rc, mb_mapping_);
    }

    // Clean up client context and socket
    modbus_close(client_ctx);
    modbus_free(client_ctx);
    close(client_socket);
  }

  void cleanupModbus()
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up Modbus resources");
    if (mb_mapping_)
    {
      modbus_mapping_free(mb_mapping_);
      mb_mapping_ = nullptr;
    }
    if (ctx_)
    {
      modbus_close(ctx_);
      modbus_free(ctx_);
      ctx_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Modbus resources cleaned up");
  }

  ~ModbusServerNode()
  {
    run_server_ = false;  // Signal all threads to stop
    cleanupModbus();
  }

private:
  std::atomic<bool> run_server_;  // Atomic flag to control server running state
  int server_socket_;
  std::mutex resource_mutex_;

  // parameters
  std::string ip_address_;
  int port_;
  int modbus_server_retry_period_ms_;
  bool log_connection_reset_;  // Flag to control logging of connection reset errors

  // modbus
  modbus_t * ctx_;
  modbus_mapping_t * mb_mapping_;
  uint16_t counter_;

  // Atomic flag to control server running state
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto modbus_server = std::make_shared<ModbusServerNode>();
  executor->add_node(modbus_server);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
