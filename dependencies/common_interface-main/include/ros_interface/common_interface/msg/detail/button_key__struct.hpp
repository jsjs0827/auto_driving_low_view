// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/ButtonKey.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__ButtonKey __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__ButtonKey __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ButtonKey_
{
  using Type = ButtonKey_<ContainerAllocator>;

  explicit ButtonKey_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->cmd = 0;
      this->pin = "";
    }
  }

  explicit ButtonKey_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pin(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->cmd = 0;
      this->pin = "";
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _pin_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pin_type pin;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__cmd(
    const uint8_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__pin(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pin = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::ButtonKey_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::ButtonKey_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::ButtonKey_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::ButtonKey_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::ButtonKey_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::ButtonKey_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::ButtonKey_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::ButtonKey_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::ButtonKey_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::ButtonKey_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__ButtonKey
    std::shared_ptr<common_interface::msg::ButtonKey_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__ButtonKey
    std::shared_ptr<common_interface::msg::ButtonKey_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ButtonKey_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->pin != other.pin) {
      return false;
    }
    return true;
  }
  bool operator!=(const ButtonKey_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ButtonKey_

// alias to use template instance with default allocator
using ButtonKey =
  common_interface::msg::ButtonKey_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__STRUCT_HPP_
