// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/McuSignal.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__McuSignal __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__McuSignal __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct McuSignal_
{
  using Type = McuSignal_<ContainerAllocator>;

  explicit McuSignal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->collision_signal = 0;
      this->suspend_signal = 0;
      this->emergency_stop_o = 0;
      this->emergency_stop_h = 0;
      this->rain_signal = 0;
      this->charging = 0;
    }
  }

  explicit McuSignal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->collision_signal = 0;
      this->suspend_signal = 0;
      this->emergency_stop_o = 0;
      this->emergency_stop_h = 0;
      this->rain_signal = 0;
      this->charging = 0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _collision_signal_type =
    uint8_t;
  _collision_signal_type collision_signal;
  using _suspend_signal_type =
    uint8_t;
  _suspend_signal_type suspend_signal;
  using _emergency_stop_o_type =
    uint8_t;
  _emergency_stop_o_type emergency_stop_o;
  using _emergency_stop_h_type =
    uint8_t;
  _emergency_stop_h_type emergency_stop_h;
  using _rain_signal_type =
    uint8_t;
  _rain_signal_type rain_signal;
  using _charging_type =
    uint8_t;
  _charging_type charging;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__collision_signal(
    const uint8_t & _arg)
  {
    this->collision_signal = _arg;
    return *this;
  }
  Type & set__suspend_signal(
    const uint8_t & _arg)
  {
    this->suspend_signal = _arg;
    return *this;
  }
  Type & set__emergency_stop_o(
    const uint8_t & _arg)
  {
    this->emergency_stop_o = _arg;
    return *this;
  }
  Type & set__emergency_stop_h(
    const uint8_t & _arg)
  {
    this->emergency_stop_h = _arg;
    return *this;
  }
  Type & set__rain_signal(
    const uint8_t & _arg)
  {
    this->rain_signal = _arg;
    return *this;
  }
  Type & set__charging(
    const uint8_t & _arg)
  {
    this->charging = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::McuSignal_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::McuSignal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::McuSignal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::McuSignal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuSignal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuSignal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuSignal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuSignal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::McuSignal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::McuSignal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__McuSignal
    std::shared_ptr<common_interface::msg::McuSignal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__McuSignal
    std::shared_ptr<common_interface::msg::McuSignal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const McuSignal_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->collision_signal != other.collision_signal) {
      return false;
    }
    if (this->suspend_signal != other.suspend_signal) {
      return false;
    }
    if (this->emergency_stop_o != other.emergency_stop_o) {
      return false;
    }
    if (this->emergency_stop_h != other.emergency_stop_h) {
      return false;
    }
    if (this->rain_signal != other.rain_signal) {
      return false;
    }
    if (this->charging != other.charging) {
      return false;
    }
    return true;
  }
  bool operator!=(const McuSignal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct McuSignal_

// alias to use template instance with default allocator
using McuSignal =
  common_interface::msg::McuSignal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__STRUCT_HPP_
