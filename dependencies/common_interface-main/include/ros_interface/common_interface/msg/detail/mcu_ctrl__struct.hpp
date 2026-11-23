// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/McuCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_CTRL__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_CTRL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__McuCtrl __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__McuCtrl __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct McuCtrl_
{
  using Type = McuCtrl_<ContainerAllocator>;

  explicit McuCtrl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->cmd = 0;
      this->status = 0;
    }
  }

  explicit McuCtrl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->cmd = 0;
      this->status = 0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _status_type =
    uint8_t;
  _status_type status;

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
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CMD_CLOSE =
    0u;
  static constexpr uint8_t CMD_REBOOT =
    1u;
  static constexpr uint8_t CMD_LOW_POWER =
    2u;
  static constexpr uint8_t CMD_MOTOR_POWER =
    3u;
  static constexpr uint8_t CMD_STOP =
    4u;
  static constexpr uint8_t CMD_FILL_LIGHT =
    5u;
  static constexpr uint8_t STATUS_CLOSE =
    0u;
  static constexpr uint8_t STATUS_OPEN =
    1u;

  // pointer types
  using RawPtr =
    common_interface::msg::McuCtrl_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::McuCtrl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::McuCtrl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::McuCtrl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuCtrl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuCtrl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuCtrl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuCtrl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::McuCtrl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::McuCtrl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__McuCtrl
    std::shared_ptr<common_interface::msg::McuCtrl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__McuCtrl
    std::shared_ptr<common_interface::msg::McuCtrl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const McuCtrl_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const McuCtrl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct McuCtrl_

// alias to use template instance with default allocator
using McuCtrl =
  common_interface::msg::McuCtrl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::CMD_CLOSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::CMD_REBOOT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::CMD_LOW_POWER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::CMD_MOTOR_POWER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::CMD_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::CMD_FILL_LIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::STATUS_CLOSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuCtrl_<ContainerAllocator>::STATUS_OPEN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_CTRL__STRUCT_HPP_
