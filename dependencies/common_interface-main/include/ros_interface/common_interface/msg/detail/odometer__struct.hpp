// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__ODOMETER__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__ODOMETER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__Odometer __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__Odometer __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Odometer_
{
  using Type = Odometer_<ContainerAllocator>;

  explicit Odometer_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->l_wheel = 0l;
      this->r_wheel = 0l;
    }
  }

  explicit Odometer_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->l_wheel = 0l;
      this->r_wheel = 0l;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _l_wheel_type =
    int32_t;
  _l_wheel_type l_wheel;
  using _r_wheel_type =
    int32_t;
  _r_wheel_type r_wheel;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__l_wheel(
    const int32_t & _arg)
  {
    this->l_wheel = _arg;
    return *this;
  }
  Type & set__r_wheel(
    const int32_t & _arg)
  {
    this->r_wheel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::Odometer_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::Odometer_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::Odometer_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::Odometer_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Odometer_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Odometer_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Odometer_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Odometer_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::Odometer_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::Odometer_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__Odometer
    std::shared_ptr<common_interface::msg::Odometer_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__Odometer
    std::shared_ptr<common_interface::msg::Odometer_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Odometer_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->l_wheel != other.l_wheel) {
      return false;
    }
    if (this->r_wheel != other.r_wheel) {
      return false;
    }
    return true;
  }
  bool operator!=(const Odometer_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Odometer_

// alias to use template instance with default allocator
using Odometer =
  common_interface::msg::Odometer_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__ODOMETER__STRUCT_HPP_
