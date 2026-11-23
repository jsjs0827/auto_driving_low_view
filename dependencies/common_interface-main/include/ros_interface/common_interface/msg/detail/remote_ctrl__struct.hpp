// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/RemoteCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__RemoteCtrl __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__RemoteCtrl __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RemoteCtrl_
{
  using Type = RemoteCtrl_<ContainerAllocator>;

  explicit RemoteCtrl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear = 0.0f;
      this->angular = 0.0f;
      this->cutter_rpm = 0;
      this->cutter_height = 0;
    }
  }

  explicit RemoteCtrl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear = 0.0f;
      this->angular = 0.0f;
      this->cutter_rpm = 0;
      this->cutter_height = 0;
    }
  }

  // field types and members
  using _linear_type =
    float;
  _linear_type linear;
  using _angular_type =
    float;
  _angular_type angular;
  using _cutter_rpm_type =
    int16_t;
  _cutter_rpm_type cutter_rpm;
  using _cutter_height_type =
    int16_t;
  _cutter_height_type cutter_height;

  // setters for named parameter idiom
  Type & set__linear(
    const float & _arg)
  {
    this->linear = _arg;
    return *this;
  }
  Type & set__angular(
    const float & _arg)
  {
    this->angular = _arg;
    return *this;
  }
  Type & set__cutter_rpm(
    const int16_t & _arg)
  {
    this->cutter_rpm = _arg;
    return *this;
  }
  Type & set__cutter_height(
    const int16_t & _arg)
  {
    this->cutter_height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::RemoteCtrl_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::RemoteCtrl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::RemoteCtrl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::RemoteCtrl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__RemoteCtrl
    std::shared_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__RemoteCtrl
    std::shared_ptr<common_interface::msg::RemoteCtrl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RemoteCtrl_ & other) const
  {
    if (this->linear != other.linear) {
      return false;
    }
    if (this->angular != other.angular) {
      return false;
    }
    if (this->cutter_rpm != other.cutter_rpm) {
      return false;
    }
    if (this->cutter_height != other.cutter_height) {
      return false;
    }
    return true;
  }
  bool operator!=(const RemoteCtrl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RemoteCtrl_

// alias to use template instance with default allocator
using RemoteCtrl =
  common_interface::msg::RemoteCtrl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__STRUCT_HPP_
