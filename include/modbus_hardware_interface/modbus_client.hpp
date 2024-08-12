
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

#ifndef MODBUS_HARDWARE_INTERFACE__MODBUS_CLIENT_HPP_
#define MODBUS_HARDWARE_INTERFACE__MODBUS_CLIENT_HPP_

#include <modbus/modbus.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modbus_hardware_interface/modbus_exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus_hardware_interface
{

constexpr char REGISTER[] = "register";
constexpr char INPUT_REGISTER[] = "input_register";
constexpr char BITS[] = "bits";
constexpr char INPUT_BITS[] = "input_bits";

class ModbusInterfaceConfig
{
public:
  explicit ModbusInterfaceConfig(
    const int & register_address, const int & number_of_bits, const double & offset = 0.0,
    const double & factor = 1.0, const bool & register_mode = false,
    const int & modbus_register_length = 16)
  : register_address_(register_address),
    number_of_bits_(number_of_bits),
    offset_(offset),
    factor_(factor),
    register_mode_(register_mode),
    modbus_register_length_(modbus_register_length)
  {
  }

  int get_register_address() const { return register_address_; }

  int number_of_bits() const { return number_of_bits_; }

  int number_of_registers() const
  {
    // if we are in register mode then the bits to read must be a multiple of the
    // modbus_register_length_
    assert(number_of_bits_ % modbus_register_length_ == 0);

    return number_of_bits_ / modbus_register_length_;
  }

  void register_mode(const bool & mode) { register_mode_ = mode; }

  bool register_mode() { return register_mode_; }

protected:
  const int register_address_;  // The address of the register
  const int number_of_bits_;    // How many bits/registers to read
  double offset_;
  double factor_;
  bool register_mode_;
  const int modbus_register_length_;
};

class ModbusInterfaceReadConfig : public ModbusInterfaceConfig
{
public:
  explicit ModbusInterfaceReadConfig(
    const int & register_address, const int & number_of_bits, const std::string & read_function,
    const std::string & data_type, const double & offset = 0.0, const double & factor = 1.0)
  : ModbusInterfaceConfig(register_address, number_of_bits, offset, factor),
    modbus_uint_16_to_float_(nullptr),
    modbus_uint_8_to_float_(nullptr),
    read_function_(read_function)
  {
    if (this->read_function() == REGISTER || this->read_function() == INPUT_REGISTER)
    {
      this->register_mode(true);
    }
    else if (this->read_function() == BITS || this->read_function() == INPUT_BITS)
    {
      this->register_mode(false);
    }
    else
    {
      throw ModbusInvalidConfigException(
        "ModbusInterfaceReadConfig: Invalid read_function passed [" + read_function_ +
        "]. Allowed types are [" + std::string(REGISTER) + "], [" + std::string(INPUT_REGISTER) +
        "], [" + std::string(BITS) + "] or [" + std::string(INPUT_BITS) + "].");
    }
    select_modbus_to_double_function(data_type);
  }

  explicit ModbusInterfaceReadConfig(
    const int & register_address, const int & number_of_bits, const std::string & read_function,
    float (*custom_modbus_uint_16_to_float)(const uint16_t *), const double & offset = 0.0,
    const double & factor = 1.0)
  : ModbusInterfaceConfig(register_address, number_of_bits, offset, factor),
    modbus_uint_16_to_float_(custom_modbus_uint_16_to_float),
    modbus_uint_8_to_float_(nullptr),
    read_function_(read_function)
  {
    // check that mode fits to given readfunction signature
    if (this->read_function() != REGISTER && this->read_function() == INPUT_REGISTER)
    {
      throw ModbusInvalidConfigException(
        "ModbusInterfaceReadConfig: Custom function passed to ModbusReadConfig with signature for "
        "reading register but read_function set to: [" +
        read_function_ + "] were [" + std::string(REGISTER) + "] or [" +
        std::string(INPUT_REGISTER) + "] is expected.");
    }
    this->register_mode(true);
  }

  explicit ModbusInterfaceReadConfig(
    const int & register_address, const int & number_of_bits, const std::string & read_function,
    float (*custom_modbus_uint_8_to_float)(const uint8_t *), const double & offset = 0.0,
    const double & factor = 1.0)
  : ModbusInterfaceConfig(register_address, number_of_bits, offset, factor),
    modbus_uint_16_to_float_(nullptr),
    modbus_uint_8_to_float_(custom_modbus_uint_8_to_float),
    read_function_(read_function)
  {
    // check that read functions type (mode) fits to given readfunction signature
    if (this->read_function() != BITS && this->read_function() != INPUT_BITS)
    {
      throw ModbusInvalidConfigException(
        "ModbusInterfaceReadConfig: Custom function passed to ModbusReadConfig with signature for "
        "reading input bits but read_function set to: [" +
        read_function_ + "] were [" + std::string(BITS) + "] or [" + std::string(INPUT_BITS) +
        "] is expected.");
    }
    this->register_mode(false);
  }

  template <typename T>
  double convert_from_modbus(const std::vector<T> & modbus_value)
  {
    return static_cast<double>(modbus_to_float(modbus_value)) * factor_ + offset_;
  }

  std::string read_function() const { return read_function_; }

protected:
  float modbus_to_float(std::vector<uint16_t> values)
  {
    THROW_ON_NULLPTR(modbus_uint_16_to_float_);
    return modbus_uint_16_to_float_(values.data());
  }

  float modbus_to_float(std::vector<uint8_t> values)
  {
    THROW_ON_NULLPTR(modbus_uint_8_to_float_);
    return modbus_uint_8_to_float_(values.data());
  }

  void select_modbus_to_double_function(const std::string & data_type)
  {
    if (register_mode())
    {
      if (data_type == "float_abcd")
      {
        modbus_uint_16_to_float_ = modbus_get_float_abcd;
      }
      else if (data_type == "float_badc")
      {
        modbus_uint_16_to_float_ = modbus_get_float_badc;
      }
      else if (data_type == "float_cdab")
      {
        modbus_uint_16_to_float_ = modbus_get_float_cdab;
      }
      else if (data_type == "float_dcba")
      {
        modbus_uint_16_to_float_ = modbus_get_float_dcba;
      }
      else if (data_type == "raw")
      {
        modbus_uint_16_to_float_ = nullptr;
      }
      else if (data_type == "to_int_to_float")
      {
        modbus_uint_16_to_float_ = [](const uint16_t * values) -> float
        {
          // Dereference the pointer to access the uint16_t values
          uint16_t firstValue = *(values);
          uint16_t secondValue = *(values + 1);
          // Shift secondValue by 16 bits to occupy the higher 16 bits of the int32_t
          int32_t result = static_cast<int32_t>(secondValue) << 16;
          // Bitwise OR with firstValue to set the lower 16 bits
          result |= static_cast<int32_t>(firstValue);
          return static_cast<float>(result);
        };
      }
      else if (data_type == "to_int_to_float_inv")
      {
        modbus_uint_16_to_float_ = [](const uint16_t * values) -> float
        {
          // Dereference the pointer to access the uint16_t values
          uint16_t firstValue = *(values);       // Equivalent to values[0]
          uint16_t secondValue = *(values + 1);  // Equivalent to values[1]

          // Shift firstValue by 16 bits to occupy the higher 16 bits of the int32_t
          int32_t result = static_cast<int32_t>(firstValue) << 16;
          // Bitwise OR with secondValue to set the lower 16 bits
          result |= static_cast<int32_t>(secondValue);
          return static_cast<float>(result);
        };
      }
      else
      {
        throw ModbusUnknownConversionFunctionException(
          "ModbusInterfaceReadConfig: Unknown modbus_uint_16_to_float_ function [" + data_type +
          "] passed.");
      }
    }
    else
    {
      if (data_type == "float")
      {
        modbus_uint_8_to_float_ = [](const uint8_t * value) -> float
        { return static_cast<float>(*value); };
      }
      else
      {
        throw ModbusUnknownConversionFunctionException(
          "ModbusInterfaceReadConfig: Unknown modbus_uint_8_to_float function [" + data_type +
          "] passed.");
      }
    }
  }
  float (*modbus_uint_16_to_float_)(const uint16_t *);  // [float_abcd, float_badc, float_dcba, ...]
  float (*modbus_uint_8_to_float_)(const uint8_t *);
  const std::string read_function_;  // register, input_register, bits, input_bits
};

class ModbusInterfaceWriteConfig : public ModbusInterfaceConfig
{
public:
  explicit ModbusInterfaceWriteConfig(
    const int & register_address, const int & number_of_number_of_bits,
    const std::string & write_function, const std::string & data_type, const double & offset = 0.0,
    const double & factor = 1.0, const bool & write_this_interface = false)
  : ModbusInterfaceConfig(register_address, number_of_number_of_bits, offset, factor),
    modbus_float_to_uint_16_(nullptr),
    modbus_float_to_uint_8_(nullptr),
    write_function_(write_function),
    write_this_interface_(write_this_interface)
  {
    if (this->write_function() == REGISTER || this->write_function() == INPUT_REGISTER)
    {
      this->register_mode(true);
    }
    else if (this->write_function() == BITS || this->write_function() == INPUT_BITS)
    {
      this->register_mode(false);
    }
    else
    {
      throw ModbusInvalidConfigException(
        "ModbusInterfaceWriteConfig: Invalid write_function passed [" + write_function_ +
        "]. Allowed types are [" + std::string(REGISTER) + "], [" + std::string(INPUT_REGISTER) +
        "], [" + std::string(BITS) + "] or [" + std::string(INPUT_BITS) + "].");
    }
    select_double_to_modbus_function(data_type);
  }

  explicit ModbusInterfaceWriteConfig(
    const int & register_address, const int & number_of_number_of_bits,
    const std::string & write_function,
    std::vector<uint16_t> (*custom_modbus_float_to_uint_16)(const float &),
    const double & offset = 0.0, const double & factor = 1.0,
    const bool & write_this_interface = false)
  : ModbusInterfaceConfig(register_address, number_of_number_of_bits, offset, factor),
    modbus_float_to_uint_16_(custom_modbus_float_to_uint_16),
    modbus_float_to_uint_8_(nullptr),
    write_function_(write_function),
    write_this_interface_(write_this_interface)
  {
    if (this->write_function() != REGISTER && this->write_function() != INPUT_REGISTER)
    {
      throw ModbusInvalidConfigException(
        "ModbusInterfaceWriteConfig: Custom function passed to ModbusInterfaceWriteConfig with "
        "signature for writing register but write_function_ set to: [" +
        write_function_ + "] were [" + std::string(REGISTER) + "] or [" +
        std::string(INPUT_REGISTER) + "] is expected.");
    }
    this->register_mode(true);
  }

  explicit ModbusInterfaceWriteConfig(
    const int & register_address, const int & number_of_number_of_bits,
    const std::string & write_function,
    std::vector<uint8_t> (*custom_modbus_float_to_uint_8)(const float &),
    const double & offset = 0.0, const double & factor = 1.0,
    const bool & write_this_interface = false)
  : ModbusInterfaceConfig(register_address, number_of_number_of_bits, offset, factor),
    modbus_float_to_uint_16_(nullptr),
    modbus_float_to_uint_8_(custom_modbus_float_to_uint_8),
    write_function_(write_function),
    write_this_interface_(write_this_interface)
  {
    if (this->write_function() != BITS && this->write_function() != INPUT_BITS)
    {
      throw ModbusInvalidConfigException(
        "ModbusInterfaceWriteConfig: Custom function passed to ModbusInterfaceWriteConfig with "
        "signature for writing bits but write_function_ set to: [" +
        write_function_ + "] were were [" + std::string(BITS) + "] or [" + std::string(INPUT_BITS) +
        "] is expected.");
    }
    this->register_mode(false);
  }

  std::vector<uint16_t> double_to_modbus_16(const double & value)
  {
    THROW_ON_NULLPTR(modbus_float_to_uint_16_);
    return modbus_float_to_uint_16_(static_cast<float>(convert(value)));
  }

  std::vector<uint8_t> double_to_modbus_8(const double & value)
  {
    THROW_ON_NULLPTR(modbus_float_to_uint_8_);
    return modbus_float_to_uint_8_(static_cast<float>(convert(value)));
  }

  std::string write_function() const { return write_function_; }

  void write_this_interface(const bool & write_this_interface)
  {
    write_this_interface_ = write_this_interface;
  }

  bool write_this_interface() const { return write_this_interface_; }

protected:
  double convert(const double & value) { return value * factor_ + offset_; }

  void select_double_to_modbus_function(const std::string & data_type)
  {
    if (register_mode())
    {
      if (data_type == "float_abcd")
      {
        modbus_float_to_uint_16_ = [](const float & value) -> std::vector<uint16_t>
        {
          std::vector<uint16_t> data(2);
          modbus_set_float_abcd(value, data.data());
          return data;
        };
      }
      else if (data_type == "float_badc")
      {
        modbus_float_to_uint_16_ = [](const float & value) -> std::vector<uint16_t>
        {
          std::vector<uint16_t> data(2);
          modbus_set_float_badc(value, data.data());
          return data;
        };
      }
      else if (data_type == "float_cdab")
      {
        modbus_float_to_uint_16_ = [](const float & value) -> std::vector<uint16_t>
        {
          std::vector<uint16_t> data(2);
          modbus_set_float_cdab(value, data.data());
          return data;
        };
      }
      else if (data_type == "raw")
      {
        modbus_float_to_uint_16_ = nullptr;
      }
      else if (data_type == "float_dcba")
      {
        modbus_float_to_uint_16_ = [](const float & value) -> std::vector<uint16_t>
        {
          std::vector<uint16_t> data(2);
          modbus_set_float_dcba(value, data.data());
          return data;
        };
      }
      else if (data_type == "to_int_to_modbus")
      {
        modbus_float_to_uint_16_ = [](const float & value) -> std::vector<uint16_t>
        {
          int32_t int_value = static_cast<uint32_t>(std::round(value));
          std::vector<uint16_t> data(2);
          // Extract the lower 16 bits (first uint16_t value)
          data[0] = static_cast<uint16_t>(int_value & 0xFFFF);
          // Extract the higher 16 bits (second uint16_t value)
          data[1] = static_cast<uint16_t>(int_value >> 16);
          return data;
        };
      }
      else if (data_type == "to_int_to_modbus_inv")
      {
        modbus_float_to_uint_16_ = [](const float & value) -> std::vector<uint16_t>
        {
          int32_t int_value = static_cast<uint32_t>(std::round(value));
          std::vector<uint16_t> data(2);
          // Extract the higher 16 bits (second uint16_t value)
          data[0] = static_cast<uint16_t>(int_value >> 16);
          // Extract the lower 16 bits (first uint16_t value)
          data[1] = static_cast<uint16_t>(int_value & 0xFFFF);
          return data;
        };
      }
      else
      {
        throw ModbusUnknownConversionFunctionException(
          "ModbusInterfaceWriteConfig: Unknown modbus_uint_16_to_float function [" + data_type +
          "] passed.");
      }
    }
    else
    {
      if (data_type == "float")
      {
        modbus_float_to_uint_8_ = [](const float & value) -> std::vector<uint8_t>
        {
          std::vector<uint8_t> data(1);
          // TODO(Manual) Maybe clamp, round and check that its zero/one?
          data.push_back(static_cast<uint8_t>(value));
          return data;
        };
      }
      else
      {
        throw ModbusUnknownConversionFunctionException(
          "ModbusInterfaceWriteConfig: Unknown modbus_8_to_float function [" + data_type +
          "] passed.");
      }
    }
  }

  std::vector<uint16_t> (*modbus_float_to_uint_16_)(const float &);
  std::vector<uint8_t> (*modbus_float_to_uint_8_)(const float &);
  const std::string write_function_;  // register, input_register, bits, input_bits
  bool write_this_interface_;
};

class ModbusClient
{
public:
  explicit ModbusClient(
    const std::string & ip_address, const int & port, const bool & persistent_connection = true,
    modbus_t * modbus_connection = nullptr)
  : ip_address_(ip_address),
    port_(port),
    persistent_connection_(persistent_connection),
    modbus_connection_(modbus_connection)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("ModbusConnection"),
      "Create connection on %s:%i with persistent connection <%s>", ip_address_.c_str(), port_,
      persistent_connection_ ? "true" : "false");
  }

  ~ModbusClient() { disconnect(); }

  bool connected() const { return modbus_connection_ != nullptr; }

  inline operator bool() const { return connected(); }

  bool is_persistent_connection() const { return persistent_connection_; }

  bool connect()
  {
    try
    {
      if (!modbus_connection_)
      {
        modbus_connection_ = modbus_new_tcp(ip_address_.c_str(), port_);
      }
      if (modbus_connection_ == NULL)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("ModbusConnection"), "Unable to create the libmodbus context");
        disconnect();
        return false;
      }

      if (modbus_connect(modbus_connection_) == -1)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("ModbusConnection"), "Connection failed: %s", modbus_strerror(errno));
        disconnect();
        return false;
      }

      RCLCPP_INFO(
        rclcpp::get_logger("ModbusConnection"), "Connected to Modbus server at %s:%d",
        ip_address_.c_str(), port_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ModbusConnection"), "Exception in connectToModbusServer: %s", e.what());
      disconnect();
      return false;
    }
    return true;
  }

  void disconnect()
  {
    if (modbus_connection_ != nullptr)
    {
      modbus_close(modbus_connection_);
      modbus_free(modbus_connection_);
      modbus_connection_ = nullptr;
    }
  }

  std::vector<uint16_t> read_raw_register(ModbusInterfaceReadConfig & config)
  {
    const std::string read_fn = config.read_function();
    if (read_fn == REGISTER)
    {
      return read_register(config.get_register_address(), config.number_of_registers());
    }
    else if (read_fn == INPUT_REGISTER)
    {
      return read_input_register(config.get_register_address(), config.number_of_registers());
    }
    else
    {
      throw std::runtime_error(
        "ModbusClient: Error while reading raw register. The given read function [" + read_fn +
        "] is not defined for registers");
    }
  }

  double read(ModbusInterfaceReadConfig & config)
  {
    const std::string read_fn = config.read_function();
    if (read_fn == REGISTER)
    {
      return config.convert_from_modbus(
        read_register(config.get_register_address(), config.number_of_registers()));
    }
    else if (read_fn == INPUT_REGISTER)
    {
      return config.convert_from_modbus(
        read_input_register(config.get_register_address(), config.number_of_registers()));
    }
    else if (read_fn == BITS)
    {
      return config.convert_from_modbus(
        read_bits(config.get_register_address(), config.number_of_bits()));
    }
    else if (read_fn == INPUT_BITS)
    {
      return config.convert_from_modbus(
        read_input_bits(config.get_register_address(), config.number_of_bits()));
    }
    else
    {
      throw std::runtime_error(
        "ModbusClient: Error while reading. The given read function [" + read_fn +
        "] is not defined.");
    }
  }

  // TODO(Manual) where should we check if there is not a value change happening?
  // Client or driver?
  void write(ModbusInterfaceWriteConfig config, const double & value)
  {
    const std::string write_fn = config.write_function();
    if (write_fn == REGISTER)
    {
      write_register(config.get_register_address(), config.double_to_modbus_16(value));
    }
    else if (write_fn == BITS)
    {
      write_bits(config.get_register_address(), config.double_to_modbus_8(value));
    }
    else
    {
      throw std::runtime_error(
        "ModbusClient: Error while writing. The given write function [" + write_fn +
        "] is not defined.");
    }
  }

private:
  std::vector<uint8_t> read_bits(const int & register_address, const int & number_of_bits = 1)
  {
    std::vector<uint8_t> bits(number_of_bits);
    int rc = modbus_read_bits(modbus_connection_, register_address, number_of_bits, bits.data());
    if (rc == -1)
    {
      std::string error_msg = "ModbusConnection: Failed to read bits in register [" +
                              std::to_string(register_address) + "]";
      RCLCPP_WARN(rclcpp::get_logger("ModbusConnection"), error_msg.c_str());

      throw ModbusReadException(error_msg);
      // just to have right default in case exception is removed
      return std::vector<uint8_t>();
    }
    return bits;
  }

  std::vector<uint8_t> read_input_bits(const int & register_address, const int & number_of_bits = 1)
  {
    std::vector<uint8_t> bits(number_of_bits);
    int rc =
      modbus_read_input_bits(modbus_connection_, register_address, number_of_bits, bits.data());
    if (rc == -1)
    {
      std::string error_msg = "ModbusConnection: Failed to read bits in register [" +
                              std::to_string(register_address) + "]";
      RCLCPP_WARN(rclcpp::get_logger("ModbusConnection"), error_msg.c_str());

      throw ModbusReadException(error_msg);
      // just to have right default in case exception is removed
      return std::vector<uint8_t>();
    }
    return bits;
  }

  std::vector<uint16_t> read_register(
    const int & register_address, const int & number_of_registers = 2)
  {
    std::vector<uint16_t> reg_dest(number_of_registers);
    int rc = modbus_read_registers(
      modbus_connection_, register_address, number_of_registers, reg_dest.data());
    if (rc == -1)
    {
      std::string error_msg =
        "ModbusConnection: Failed to read register [" + std::to_string(register_address) + "]";
      RCLCPP_WARN(rclcpp::get_logger("ModbusConnection"), error_msg.c_str());

      throw ModbusReadException(error_msg);
      // just to have right default in case exception is removed
      return std::vector<uint16_t>();
    }
    return reg_dest;
  }

  std::vector<uint16_t> read_input_register(
    const int & register_address, const int & number_of_registers = 2)
  {
    std::vector<uint16_t> reg_dest(number_of_registers);
    int rc = modbus_read_input_registers(
      modbus_connection_, register_address, number_of_registers, reg_dest.data());
    if (rc == -1)
    {
      std::string error_msg = "ModbusConnection: Failed to read input register [" +
                              std::to_string(register_address) + "]";
      RCLCPP_WARN(rclcpp::get_logger("ModbusConnection"), error_msg.c_str());

      throw ModbusReadException(error_msg);
      // just to have right default in case exception is removed
      return std::vector<uint16_t>();
    }
    return reg_dest;
  }

  int write_bits(const int & register_address, const std::vector<uint8_t> & bits)
  {
    // static cast should not pose a problem here since its very unlikely that we want to read more
    // than 2^31 bits
    int wc = modbus_write_bits(
      modbus_connection_, register_address, static_cast<int>(bits.size()), bits.data());
    if (wc == -1)
    {
      std::string error_msg = "ModbusConnection: Failed to write bits in register [" +
                              std::to_string(register_address) + "]";
      RCLCPP_WARN(rclcpp::get_logger("ModbusConnection"), error_msg.c_str());

      throw ModbusWriteException(error_msg);
      // just to have right default in case exception is removed
      return wc;
    }
    return wc;
  }

  int write_register(const int & register_address, std::vector<uint16_t> values)
  {
    int wc = modbus_write_registers(
      modbus_connection_, register_address, static_cast<int>(values.size()), values.data());
    if (wc == -1)
    {
      std::string error_msg =
        "ModbusConnection: Failed to write register [" + std::to_string(register_address) + "]";
      RCLCPP_WARN(rclcpp::get_logger("ModbusConnection"), error_msg.c_str());

      throw ModbusWriteException(error_msg);
      // just to have right default in case exception is removed
      return wc;
    }
    return wc;
  }

  // Private members
  std::string ip_address_;
  int port_;
  bool persistent_connection_;
  modbus_t * modbus_connection_;
};

}  // namespace modbus_hardware_interface

#endif  // MODBUS_HARDWARE_INTERFACE__MODBUS_CLIENT_HPP_
