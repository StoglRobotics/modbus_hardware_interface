# ModbusHardwareInterface
If you not familiar with `ros2_control.xacro` check the [section at the end](#mini-urdf-and-xacro-introduction).
## Using ModbusHardwareInterface in ros2_control.xacro
In this files we define the hardware components like the servo, linear actuator and proximity sensor. You can see there which component (plugin) is used and how the component is configured.

Let's make an example: If we want to use the `ModbusHardwareInterface` in our project we can create a `my_example_hw.ros2_control.xacro`. In this file we define the used implementation with the `<plugin>...</plugin>` tag:
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:macro name="my_servo_motor_ros2_control" params="
               name
               modbus_server_ip:=10.150.1.4
               modbus_server_port:=502
               modbus_use_persistent_connection:=true"
               >
    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>modbus_hardware_interface/ModbusHardwareInterface</plugin>
        <param name="modbus_server_ip">${modbus_server_ip}</param>
        <param name="modbus_server_port">${modbus_server_port}</param>
        <param name="use_persistent_connection">${modbus_use_persistent_connection}</param>
      </hardware>
  </xacro:macro>
</robot>
```
The plugin itself is defined in the `modbus_hardware_interface.xml` file. Additionally we set the ip and port for the modbus server there.

Besides this the most important part of this files are that we define the provided *Command-/StateInterfaces* of the hardware component.

### Creation of Command-/StateInterfaces
To define a command or state interface you have to supply following parameters for modbus:\
**CommandInterface:**
```
<command_interface name="name_of_interface">
    <param name="register">number_of_register*2</param>
    <param name="bits_to_read"number of bits to write/register size</param>
    <param name="conversion_fn">how to convert from double to modbus int</param>
    <param name="write_function">what modbus type</param>
    <param name="factor">for converting to modbus</param>
    <param name="offset">for converting to modbus</param>
</command_interface>
```
**StateInterface:**
```
<state_interface name="position">
    <param name="register">umber_of_register*2</param>
    <param name="bits_to_read">number of bits to read/register size</param>
    <param name="conversion_fn">how to convert from modbus int to double</param>
    <param name="read_function">what modbus type</param>
    <param name="factor">for converting from modbus</param>
    <param name="offset">or converting from modbus</param>
</state_interface>
```
#### Available xml parameters:
The `ModbusHardwareInterface` supports the following parameters to configure how a StateInterface acquires its value from modbus or how a CommandInterface writes its value to modbus.
##### register (mandatory)
Register number. Defines which register is requester for reading or writing. Check data sheet of your hardware for reference.
##### bits_to_read (mandatory)
How many bits we read/write when accessing this modbus register. For registers this needs to be the register size and a multiple of `sizeof(uint_16t)`. If the register is not a multiple of 16, which is the underlying modbus int size, this could be problematic. For bit it says how many bits are read/written in one call.
##### conversion_fn
For reading: This maps to a function which is used to convert from uint_16t (register) or uint_8t (bits) used in modbus to double used in ros2_control. You can check the modbus [library](https://libmodbus.org/reference/) to see which functions are available and the implementations in [modbus_client.hpp](https://github.com/StoglRobotics/modbus_hardware_interface/blob/master/include/modbus_hardware_interface/modbus_client.hpp). If you need some special conversion you have to implement.\
For writing: This maps to a function which is used to convert from double to uint_16t (register) or uint_8t (bits).
##### read_function/write_function (mandatory)
read_function is for StatateIntefaces only, write_functions for CommandInterfaces only. \
We distinguish between reading a register or single bits. This is done by setting the *read_function/write_function*:
* register -> read/write register
* input_register -> used for read only registers
* bits -> read/write bits
* input_bits -> read only bits
##### factor and offset
Used for converting from the special modbus datatype to double. This is different then a conversion from uint_8t/uint_16 to double. With the conversion_fn we just convert from uint_8tuint_16 to double. For example the consider the position StateInterface is given in ticks but we want to provide a position in m in our StateInterface. Therefore we have to convert the double we receive by simply converting from modbus to double further. Therefore we supply the options of giving a factor and offset-> factor*converted_uint_16+offset.\
The same is for the conversion from ros2_control double -> modbus uint_16 true.

## Using ModbusHardwareInterface in ros2_control.xacro
If you need a custom HardwareInterface you first have to inherit from the `ModbusHardwareInterface`:
```
class MyServo : public modbus_hardware_interface::ModbusHardwareInterface
```
then override the functions you need some custom actions. Important! Don't forget to call the `ModbusHardwareInterface` functions in your implementations e.g. for: `on_init`, `read`, `write` like this

```
hardware_interface::CallbackReturn ServoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (modbus_hardware_interface::ModbusHardwareInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  ...
}
```
```
hardware_interface::return_type ServoHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = modbus_hardware_interface::ModbusHardwareInterface::read(time, period);
  if (ret != hardware_interface::return_type::OK)
  {
    return ret;
  }
  .....
}
```
If you do not call those the modbus specific parameters in `ros2_control.xacro` will not be parsed and the StateInterfaces will not be read or CommandInterfaces be written.

# Mini URDF and Xacro introduction
## URDF
URDF, Unified Robot Description Format is an XML format for representing a robot model. URDF is commonly used in Robot Operating System (ROS) tools such as rviz (Ros Visualization tool) and Gazebo simulator. The model consists of links and joints motion.
## [Xacro](https://github.com/ros/xacro/wiki)
Xacro is an XML macro language and preprocessor that is typically used to simplify the generation of URDF files. However, it can be applied to any XML. Using parameterizable macros, re-occurring XML blocks can be created with a few commands only.
## ros2_control.xacro
Check the [control docs](https://control.ros.org/rolling/doc/getting_started/getting_started.html#hardware-description-in-urdf)
# References
* [https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html)
* [https://github.com/ros/xacro/wiki](https://github.com/ros/xacro/wiki)
* [https://wiki.ros.org/xacro](https://wiki.ros.org/xacro)
