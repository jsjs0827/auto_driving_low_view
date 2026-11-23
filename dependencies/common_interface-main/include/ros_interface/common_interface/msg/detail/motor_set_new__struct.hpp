// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/MotorSetNew.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET_NEW__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET_NEW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__MotorSetNew __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__MotorSetNew __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSetNew_
{
  using Type = MotorSetNew_<ContainerAllocator>;

  explicit MotorSetNew_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->left_front_wheel = 0;
      this->right_front_wheel = 0;
      this->left_back_wheel = 0;
      this->right_back_wheel = 0;
      this->left_angle = 0;
      this->right_angle = 0;
      this->cutter_rpm = 0;
      this->cutting_edge = 0;
      this->adjust_motor = 0;
      this->sequence = 0;
    }
  }

  explicit MotorSetNew_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->left_front_wheel = 0;
      this->right_front_wheel = 0;
      this->left_back_wheel = 0;
      this->right_back_wheel = 0;
      this->left_angle = 0;
      this->right_angle = 0;
      this->cutter_rpm = 0;
      this->cutting_edge = 0;
      this->adjust_motor = 0;
      this->sequence = 0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _left_front_wheel_type =
    int16_t;
  _left_front_wheel_type left_front_wheel;
  using _right_front_wheel_type =
    int16_t;
  _right_front_wheel_type right_front_wheel;
  using _left_back_wheel_type =
    int16_t;
  _left_back_wheel_type left_back_wheel;
  using _right_back_wheel_type =
    int16_t;
  _right_back_wheel_type right_back_wheel;
  using _left_angle_type =
    int16_t;
  _left_angle_type left_angle;
  using _right_angle_type =
    int16_t;
  _right_angle_type right_angle;
  using _cutter_rpm_type =
    int16_t;
  _cutter_rpm_type cutter_rpm;
  using _cutting_edge_type =
    int16_t;
  _cutting_edge_type cutting_edge;
  using _adjust_motor_type =
    int16_t;
  _adjust_motor_type adjust_motor;
  using _sequence_type =
    int16_t;
  _sequence_type sequence;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__left_front_wheel(
    const int16_t & _arg)
  {
    this->left_front_wheel = _arg;
    return *this;
  }
  Type & set__right_front_wheel(
    const int16_t & _arg)
  {
    this->right_front_wheel = _arg;
    return *this;
  }
  Type & set__left_back_wheel(
    const int16_t & _arg)
  {
    this->left_back_wheel = _arg;
    return *this;
  }
  Type & set__right_back_wheel(
    const int16_t & _arg)
  {
    this->right_back_wheel = _arg;
    return *this;
  }
  Type & set__left_angle(
    const int16_t & _arg)
  {
    this->left_angle = _arg;
    return *this;
  }
  Type & set__right_angle(
    const int16_t & _arg)
  {
    this->right_angle = _arg;
    return *this;
  }
  Type & set__cutter_rpm(
    const int16_t & _arg)
  {
    this->cutter_rpm = _arg;
    return *this;
  }
  Type & set__cutting_edge(
    const int16_t & _arg)
  {
    this->cutting_edge = _arg;
    return *this;
  }
  Type & set__adjust_motor(
    const int16_t & _arg)
  {
    this->adjust_motor = _arg;
    return *this;
  }
  Type & set__sequence(
    const int16_t & _arg)
  {
    this->sequence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::MotorSetNew_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::MotorSetNew_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MotorSetNew_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MotorSetNew_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__MotorSetNew
    std::shared_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__MotorSetNew
    std::shared_ptr<common_interface::msg::MotorSetNew_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSetNew_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->left_front_wheel != other.left_front_wheel) {
      return false;
    }
    if (this->right_front_wheel != other.right_front_wheel) {
      return false;
    }
    if (this->left_back_wheel != other.left_back_wheel) {
      return false;
    }
    if (this->right_back_wheel != other.right_back_wheel) {
      return false;
    }
    if (this->left_angle != other.left_angle) {
      return false;
    }
    if (this->right_angle != other.right_angle) {
      return false;
    }
    if (this->cutter_rpm != other.cutter_rpm) {
      return false;
    }
    if (this->cutting_edge != other.cutting_edge) {
      return false;
    }
    if (this->adjust_motor != other.adjust_motor) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSetNew_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSetNew_

// alias to use template instance with default allocator
using MotorSetNew =
  common_interface::msg::MotorSetNew_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET_NEW__STRUCT_HPP_
