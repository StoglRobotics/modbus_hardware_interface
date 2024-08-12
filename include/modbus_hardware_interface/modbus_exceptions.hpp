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

#ifndef MODBUS_HARDWARE_INTERFACE__MODBUS_EXCEPTIONS_HPP_
#define MODBUS_HARDWARE_INTERFACE__MODBUS_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace modbus_hardware_interface
{

class ModbusReadException : public std::runtime_error
{
public:
  explicit ModbusReadException(const std::string & message) : std::runtime_error(message) {}

  virtual ~ModbusReadException() noexcept {}
};

class ModbusWriteException : public std::runtime_error
{
public:
  explicit ModbusWriteException(const std::string & message) : std::runtime_error(message) {}

  virtual ~ModbusWriteException() noexcept {}
};

class ModbusConnectionException : public std::runtime_error
{
public:
  explicit ModbusConnectionException(const std::string & message) : std::runtime_error(message) {}

  virtual ~ModbusConnectionException() noexcept {}
};

class ModbusInvalidConfigException : public std::runtime_error
{
public:
  explicit ModbusInvalidConfigException(const std::string & message) : std::runtime_error(message)
  {
  }

  virtual ~ModbusInvalidConfigException() noexcept {}
};

class ModbusUnknownConversionFunctionException : public std::runtime_error
{
public:
  explicit ModbusUnknownConversionFunctionException(const std::string & message)
  : std::runtime_error(message)
  {
  }

  virtual ~ModbusUnknownConversionFunctionException() noexcept {}
};

}  // namespace modbus_hardware_interface

#endif  // MODBUS_HARDWARE_INTERFACE__MODBUS_EXCEPTIONS_HPP_
