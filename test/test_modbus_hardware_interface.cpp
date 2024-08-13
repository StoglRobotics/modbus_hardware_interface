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

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestModbusHardwareInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    valid_modbus_hardware =
      R"(
        <ros2_control name="ModbusHardwareInterface2dof" type="system">
          <hardware>
            <plugin>modbus_hardware_interface/ModbusHardwareInterface</plugin>
            <param name="modbus_server_ip">127.0.0.1</param>
            <param name="modbus_server_port">1234</param>
            <param name="use_persistent_connection">true</param>
          </hardware>
          <joint name="joint1">
            <command_interface name="position">
              <param name="register">1</param>
              <param name="bits_to_read">32</param>
              <param name="conversion_fn">float_abcd</param>
              <param name="write_function">register</param>
            </command_interface>
            <command_interface name="velocity">
              <param name="register">2</param>
              <param name="bits_to_read">1</param>
              <param name="conversion_fn">float</param>
              <param name="write_function">bits</param>
            </command_interface>

            <state_interface name="position">
              <param name="register">2</param>
              <param name="bits_to_read">1</param>
              <param name="conversion_fn">float</param>
              <param name="read_function">input_bits</param>
            </state_interface>
            <state_interface name="velocity">
              <param name="register">32</param>
              <param name="bits_to_read">32</param>
              <param name="conversion_fn">float_abcd</param>
              <param name="read_function">input_register</param>
            </state_interface>
          </joint>
        </ros2_control>
    )";
  }
  std::string valid_modbus_hardware;
};

TEST_F(TestModbusHardwareInterface, load_modbus_hardware_interface_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + valid_modbus_hardware +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
