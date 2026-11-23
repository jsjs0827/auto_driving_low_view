// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/Bat.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__BAT__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__BAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__Bat __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__Bat __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bat_
{
  using Type = Bat_<ContainerAllocator>;

  explicit Bat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0;
      this->capacity = 0;
      this->series_num = 0;
      this->parallel_num = 0;
      this->real_voltage = 0;
      this->single_voltage = 0;
      this->temperature = 0;
      this->power = 0;
      this->code_value = 0;
      this->sequence = 0;
    }
  }

  explicit Bat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0;
      this->capacity = 0;
      this->series_num = 0;
      this->parallel_num = 0;
      this->real_voltage = 0;
      this->single_voltage = 0;
      this->temperature = 0;
      this->power = 0;
      this->code_value = 0;
      this->sequence = 0;
    }
  }

  // field types and members
  using _voltage_type =
    int16_t;
  _voltage_type voltage;
  using _capacity_type =
    uint8_t;
  _capacity_type capacity;
  using _series_num_type =
    uint8_t;
  _series_num_type series_num;
  using _parallel_num_type =
    uint8_t;
  _parallel_num_type parallel_num;
  using _real_voltage_type =
    int16_t;
  _real_voltage_type real_voltage;
  using _single_voltage_type =
    int16_t;
  _single_voltage_type single_voltage;
  using _temperature_type =
    int8_t;
  _temperature_type temperature;
  using _power_type =
    uint8_t;
  _power_type power;
  using _code_value_type =
    uint8_t;
  _code_value_type code_value;
  using _sequence_type =
    uint16_t;
  _sequence_type sequence;

  // setters for named parameter idiom
  Type & set__voltage(
    const int16_t & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__capacity(
    const uint8_t & _arg)
  {
    this->capacity = _arg;
    return *this;
  }
  Type & set__series_num(
    const uint8_t & _arg)
  {
    this->series_num = _arg;
    return *this;
  }
  Type & set__parallel_num(
    const uint8_t & _arg)
  {
    this->parallel_num = _arg;
    return *this;
  }
  Type & set__real_voltage(
    const int16_t & _arg)
  {
    this->real_voltage = _arg;
    return *this;
  }
  Type & set__single_voltage(
    const int16_t & _arg)
  {
    this->single_voltage = _arg;
    return *this;
  }
  Type & set__temperature(
    const int8_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__power(
    const uint8_t & _arg)
  {
    this->power = _arg;
    return *this;
  }
  Type & set__code_value(
    const uint8_t & _arg)
  {
    this->code_value = _arg;
    return *this;
  }
  Type & set__sequence(
    const uint16_t & _arg)
  {
    this->sequence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::Bat_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::Bat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::Bat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::Bat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Bat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Bat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Bat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Bat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::Bat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::Bat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__Bat
    std::shared_ptr<common_interface::msg::Bat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__Bat
    std::shared_ptr<common_interface::msg::Bat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bat_ & other) const
  {
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->capacity != other.capacity) {
      return false;
    }
    if (this->series_num != other.series_num) {
      return false;
    }
    if (this->parallel_num != other.parallel_num) {
      return false;
    }
    if (this->real_voltage != other.real_voltage) {
      return false;
    }
    if (this->single_voltage != other.single_voltage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->power != other.power) {
      return false;
    }
    if (this->code_value != other.code_value) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bat_

// alias to use template instance with default allocator
using Bat =
  common_interface::msg::Bat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__BAT__STRUCT_HPP_
