// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/McuOtaStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_OTA_STATUS__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_OTA_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__McuOtaStatus __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__McuOtaStatus __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct McuOtaStatus_
{
  using Type = McuOtaStatus_<ContainerAllocator>;

  explicit McuOtaStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->status = 0;
      this->progress = 0;
    }
  }

  explicit McuOtaStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->status = 0;
      this->progress = 0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _status_type =
    uint8_t;
  _status_type status;
  using _progress_type =
    uint8_t;
  _progress_type progress;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__progress(
    const uint8_t & _arg)
  {
    this->progress = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATUS_RUNNING =
    0u;
  static constexpr uint8_t STATUS_FAILED =
    1u;
  static constexpr uint8_t STATUS_COMPLETE =
    2u;

  // pointer types
  using RawPtr =
    common_interface::msg::McuOtaStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::McuOtaStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuOtaStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::McuOtaStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__McuOtaStatus
    std::shared_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__McuOtaStatus
    std::shared_ptr<common_interface::msg::McuOtaStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const McuOtaStatus_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->progress != other.progress) {
      return false;
    }
    return true;
  }
  bool operator!=(const McuOtaStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct McuOtaStatus_

// alias to use template instance with default allocator
using McuOtaStatus =
  common_interface::msg::McuOtaStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuOtaStatus_<ContainerAllocator>::STATUS_RUNNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuOtaStatus_<ContainerAllocator>::STATUS_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t McuOtaStatus_<ContainerAllocator>::STATUS_COMPLETE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_OTA_STATUS__STRUCT_HPP_
