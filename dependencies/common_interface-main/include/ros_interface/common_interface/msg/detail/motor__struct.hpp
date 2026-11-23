// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/Motor.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__Motor __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__Motor __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Motor_
{
  using Type = Motor_<ContainerAllocator>;

  explicit Motor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->left_wheel = 0;
      this->right_wheel = 0;
      this->cutter_rpm = 0;
      this->lwheel_current = 0;
      this->rwheel_current = 0;
      this->cutter_current = 0;
      this->lwheel_code_value = 0;
      this->rwheel_code_value = 0;
      this->cutter_code_value = 0;
      this->adjust_status = 0;
      this->cutting_edge_speed = 0;
      this->sequence = 0;
    }
  }

  explicit Motor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->left_wheel = 0;
      this->right_wheel = 0;
      this->cutter_rpm = 0;
      this->lwheel_current = 0;
      this->rwheel_current = 0;
      this->cutter_current = 0;
      this->lwheel_code_value = 0;
      this->rwheel_code_value = 0;
      this->cutter_code_value = 0;
      this->adjust_status = 0;
      this->cutting_edge_speed = 0;
      this->sequence = 0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _left_wheel_type =
    int16_t;
  _left_wheel_type left_wheel;
  using _right_wheel_type =
    int16_t;
  _right_wheel_type right_wheel;
  using _cutter_rpm_type =
    int16_t;
  _cutter_rpm_type cutter_rpm;
  using _lwheel_current_type =
    int16_t;
  _lwheel_current_type lwheel_current;
  using _rwheel_current_type =
    int16_t;
  _rwheel_current_type rwheel_current;
  using _cutter_current_type =
    int16_t;
  _cutter_current_type cutter_current;
  using _lwheel_code_value_type =
    uint16_t;
  _lwheel_code_value_type lwheel_code_value;
  using _rwheel_code_value_type =
    uint16_t;
  _rwheel_code_value_type rwheel_code_value;
  using _cutter_code_value_type =
    uint16_t;
  _cutter_code_value_type cutter_code_value;
  using _adjust_status_type =
    uint8_t;
  _adjust_status_type adjust_status;
  using _cutting_edge_speed_type =
    uint16_t;
  _cutting_edge_speed_type cutting_edge_speed;
  using _sequence_type =
    uint16_t;
  _sequence_type sequence;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__left_wheel(
    const int16_t & _arg)
  {
    this->left_wheel = _arg;
    return *this;
  }
  Type & set__right_wheel(
    const int16_t & _arg)
  {
    this->right_wheel = _arg;
    return *this;
  }
  Type & set__cutter_rpm(
    const int16_t & _arg)
  {
    this->cutter_rpm = _arg;
    return *this;
  }
  Type & set__lwheel_current(
    const int16_t & _arg)
  {
    this->lwheel_current = _arg;
    return *this;
  }
  Type & set__rwheel_current(
    const int16_t & _arg)
  {
    this->rwheel_current = _arg;
    return *this;
  }
  Type & set__cutter_current(
    const int16_t & _arg)
  {
    this->cutter_current = _arg;
    return *this;
  }
  Type & set__lwheel_code_value(
    const uint16_t & _arg)
  {
    this->lwheel_code_value = _arg;
    return *this;
  }
  Type & set__rwheel_code_value(
    const uint16_t & _arg)
  {
    this->rwheel_code_value = _arg;
    return *this;
  }
  Type & set__cutter_code_value(
    const uint16_t & _arg)
  {
    this->cutter_code_value = _arg;
    return *this;
  }
  Type & set__adjust_status(
    const uint8_t & _arg)
  {
    this->adjust_status = _arg;
    return *this;
  }
  Type & set__cutting_edge_speed(
    const uint16_t & _arg)
  {
    this->cutting_edge_speed = _arg;
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
    common_interface::msg::Motor_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::Motor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::Motor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::Motor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Motor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Motor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Motor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Motor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::Motor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::Motor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__Motor
    std::shared_ptr<common_interface::msg::Motor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__Motor
    std::shared_ptr<common_interface::msg::Motor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Motor_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->left_wheel != other.left_wheel) {
      return false;
    }
    if (this->right_wheel != other.right_wheel) {
      return false;
    }
    if (this->cutter_rpm != other.cutter_rpm) {
      return false;
    }
    if (this->lwheel_current != other.lwheel_current) {
      return false;
    }
    if (this->rwheel_current != other.rwheel_current) {
      return false;
    }
    if (this->cutter_current != other.cutter_current) {
      return false;
    }
    if (this->lwheel_code_value != other.lwheel_code_value) {
      return false;
    }
    if (this->rwheel_code_value != other.rwheel_code_value) {
      return false;
    }
    if (this->cutter_code_value != other.cutter_code_value) {
      return false;
    }
    if (this->adjust_status != other.adjust_status) {
      return false;
    }
    if (this->cutting_edge_speed != other.cutting_edge_speed) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Motor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Motor_

// alias to use template instance with default allocator
using Motor =
  common_interface::msg::Motor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR__STRUCT_HPP_
