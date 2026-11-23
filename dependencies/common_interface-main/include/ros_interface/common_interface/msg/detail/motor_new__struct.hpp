// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/MotorNew.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__MotorNew __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__MotorNew __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorNew_
{
  using Type = MotorNew_<ContainerAllocator>;

  explicit MotorNew_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->lfwheel_current = 0;
      this->rfwheel_current = 0;
      this->lbwheel_current = 0;
      this->rbwheel_current = 0;
      this->left_turn_current = 0;
      this->right_turn_current = 0;
      this->cutter_current = 0;
      this->lfwheel_code_value = 0;
      this->rfwheel_code_value = 0;
      this->lbwheel_code_value = 0;
      this->rbwheel_code_value = 0;
      this->lturn_code_value = 0;
      this->rturn_code_value = 0;
      this->cutter_code_value = 0;
      this->adjust_status = 0;
      this->cutting_edge_speed = 0;
      this->sequence = 0;
    }
  }

  explicit MotorNew_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->lfwheel_current = 0;
      this->rfwheel_current = 0;
      this->lbwheel_current = 0;
      this->rbwheel_current = 0;
      this->left_turn_current = 0;
      this->right_turn_current = 0;
      this->cutter_current = 0;
      this->lfwheel_code_value = 0;
      this->rfwheel_code_value = 0;
      this->lbwheel_code_value = 0;
      this->rbwheel_code_value = 0;
      this->lturn_code_value = 0;
      this->rturn_code_value = 0;
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
  using _lfwheel_current_type =
    int16_t;
  _lfwheel_current_type lfwheel_current;
  using _rfwheel_current_type =
    int16_t;
  _rfwheel_current_type rfwheel_current;
  using _lbwheel_current_type =
    int16_t;
  _lbwheel_current_type lbwheel_current;
  using _rbwheel_current_type =
    int16_t;
  _rbwheel_current_type rbwheel_current;
  using _left_turn_current_type =
    int16_t;
  _left_turn_current_type left_turn_current;
  using _right_turn_current_type =
    int16_t;
  _right_turn_current_type right_turn_current;
  using _cutter_current_type =
    int16_t;
  _cutter_current_type cutter_current;
  using _lfwheel_code_value_type =
    uint16_t;
  _lfwheel_code_value_type lfwheel_code_value;
  using _rfwheel_code_value_type =
    uint16_t;
  _rfwheel_code_value_type rfwheel_code_value;
  using _lbwheel_code_value_type =
    uint16_t;
  _lbwheel_code_value_type lbwheel_code_value;
  using _rbwheel_code_value_type =
    uint16_t;
  _rbwheel_code_value_type rbwheel_code_value;
  using _lturn_code_value_type =
    uint16_t;
  _lturn_code_value_type lturn_code_value;
  using _rturn_code_value_type =
    uint16_t;
  _rturn_code_value_type rturn_code_value;
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
  Type & set__lfwheel_current(
    const int16_t & _arg)
  {
    this->lfwheel_current = _arg;
    return *this;
  }
  Type & set__rfwheel_current(
    const int16_t & _arg)
  {
    this->rfwheel_current = _arg;
    return *this;
  }
  Type & set__lbwheel_current(
    const int16_t & _arg)
  {
    this->lbwheel_current = _arg;
    return *this;
  }
  Type & set__rbwheel_current(
    const int16_t & _arg)
  {
    this->rbwheel_current = _arg;
    return *this;
  }
  Type & set__left_turn_current(
    const int16_t & _arg)
  {
    this->left_turn_current = _arg;
    return *this;
  }
  Type & set__right_turn_current(
    const int16_t & _arg)
  {
    this->right_turn_current = _arg;
    return *this;
  }
  Type & set__cutter_current(
    const int16_t & _arg)
  {
    this->cutter_current = _arg;
    return *this;
  }
  Type & set__lfwheel_code_value(
    const uint16_t & _arg)
  {
    this->lfwheel_code_value = _arg;
    return *this;
  }
  Type & set__rfwheel_code_value(
    const uint16_t & _arg)
  {
    this->rfwheel_code_value = _arg;
    return *this;
  }
  Type & set__lbwheel_code_value(
    const uint16_t & _arg)
  {
    this->lbwheel_code_value = _arg;
    return *this;
  }
  Type & set__rbwheel_code_value(
    const uint16_t & _arg)
  {
    this->rbwheel_code_value = _arg;
    return *this;
  }
  Type & set__lturn_code_value(
    const uint16_t & _arg)
  {
    this->lturn_code_value = _arg;
    return *this;
  }
  Type & set__rturn_code_value(
    const uint16_t & _arg)
  {
    this->rturn_code_value = _arg;
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
    common_interface::msg::MotorNew_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::MotorNew_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::MotorNew_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::MotorNew_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MotorNew_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MotorNew_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MotorNew_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MotorNew_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::MotorNew_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::MotorNew_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__MotorNew
    std::shared_ptr<common_interface::msg::MotorNew_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__MotorNew
    std::shared_ptr<common_interface::msg::MotorNew_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorNew_ & other) const
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
    if (this->lfwheel_current != other.lfwheel_current) {
      return false;
    }
    if (this->rfwheel_current != other.rfwheel_current) {
      return false;
    }
    if (this->lbwheel_current != other.lbwheel_current) {
      return false;
    }
    if (this->rbwheel_current != other.rbwheel_current) {
      return false;
    }
    if (this->left_turn_current != other.left_turn_current) {
      return false;
    }
    if (this->right_turn_current != other.right_turn_current) {
      return false;
    }
    if (this->cutter_current != other.cutter_current) {
      return false;
    }
    if (this->lfwheel_code_value != other.lfwheel_code_value) {
      return false;
    }
    if (this->rfwheel_code_value != other.rfwheel_code_value) {
      return false;
    }
    if (this->lbwheel_code_value != other.lbwheel_code_value) {
      return false;
    }
    if (this->rbwheel_code_value != other.rbwheel_code_value) {
      return false;
    }
    if (this->lturn_code_value != other.lturn_code_value) {
      return false;
    }
    if (this->rturn_code_value != other.rturn_code_value) {
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
  bool operator!=(const MotorNew_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorNew_

// alias to use template instance with default allocator
using MotorNew =
  common_interface::msg::MotorNew_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__STRUCT_HPP_
