// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/McuOta.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_OTA__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_OTA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__McuOta __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__McuOta __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct McuOta_
{
  using Type = McuOta_<ContainerAllocator>;

  explicit McuOta_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->cmd = 0;
      this->address = "";
    }
  }

  explicit McuOta_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : address(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->cmd = 0;
      this->address = "";
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _address_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _address_type address;

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
  Type & set__address(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->address = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CMD_ENTER_OTA =
    1u;
  static constexpr uint8_t CMD_REBOOT =
    2u;

  // pointer types
  using RawPtr =
    common_interface::msg::McuOta_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::McuOta_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::McuOta_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::McuOta_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuOta_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuOta_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuOta_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuOta_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::McuOta_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::McuOta_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__McuOta
    std::shared_ptr<common_interface::msg::McuOta_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__McuOta
    std::shared_ptr<common_interface::msg::McuOta_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const McuOta_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->address != other.address) {
      return false;
    }
    return true;
  }
  bool operator!=(const McuOta_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct McuOta_

// alias to use template instance with default allocator
using McuOta =
  common_interface::msg::McuOta_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuOta_<ContainerAllocator>::CMD_ENTER_OTA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuOta_<ContainerAllocator>::CMD_REBOOT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_OTA__STRUCT_HPP_
