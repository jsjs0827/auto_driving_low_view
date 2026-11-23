// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__MowerStatus __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__MowerStatus __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerStatus_
{
  using Type = MowerStatus_<ContainerAllocator>;

  explicit MowerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MowerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATUS_UNKNOWN =
    0u;
  static constexpr uint8_t STATUS_RETURNING_TO_DOCK =
    1u;
  static constexpr uint8_t STATUS_MAPPING =
    2u;
  static constexpr uint8_t STATUS_MOWING =
    3u;

  // pointer types
  using RawPtr =
    common_interface::msg::MowerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::MowerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::MowerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::MowerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MowerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MowerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MowerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MowerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::MowerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::MowerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__MowerStatus
    std::shared_ptr<common_interface::msg::MowerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__MowerStatus
    std::shared_ptr<common_interface::msg::MowerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerStatus_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerStatus_

// alias to use template instance with default allocator
using MowerStatus =
  common_interface::msg::MowerStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MowerStatus_<ContainerAllocator>::STATUS_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MowerStatus_<ContainerAllocator>::STATUS_RETURNING_TO_DOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MowerStatus_<ContainerAllocator>::STATUS_MAPPING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MowerStatus_<ContainerAllocator>::STATUS_MOWING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_
