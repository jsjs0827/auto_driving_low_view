// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/M7xxFirmwareUpgradeProgress.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__M7XX_FIRMWARE_UPGRADE_PROGRESS__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__M7XX_FIRMWARE_UPGRADE_PROGRESS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__M7xxFirmwareUpgradeProgress __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__M7xxFirmwareUpgradeProgress __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct M7xxFirmwareUpgradeProgress_
{
  using Type = M7xxFirmwareUpgradeProgress_<ContainerAllocator>;

  explicit M7xxFirmwareUpgradeProgress_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->percent = 0;
      this->stage = 0;
      this->result_code = 0l;
      this->message = "";
    }
  }

  explicit M7xxFirmwareUpgradeProgress_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->percent = 0;
      this->stage = 0;
      this->result_code = 0l;
      this->message = "";
    }
  }

  // field types and members
  using _percent_type =
    uint8_t;
  _percent_type percent;
  using _stage_type =
    uint8_t;
  _stage_type stage;
  using _result_code_type =
    int32_t;
  _result_code_type result_code;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__percent(
    const uint8_t & _arg)
  {
    this->percent = _arg;
    return *this;
  }
  Type & set__stage(
    const uint8_t & _arg)
  {
    this->stage = _arg;
    return *this;
  }
  Type & set__result_code(
    const int32_t & _arg)
  {
    this->result_code = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STAGE_PREPARING =
    0u;
  static constexpr uint8_t STAGE_TRANSFERRING =
    1u;
  static constexpr uint8_t STAGE_RETRY_WAIT =
    2u;
  static constexpr uint8_t STAGE_RECOVERING =
    3u;
  static constexpr uint8_t STAGE_SUCCEEDED =
    4u;
  static constexpr uint8_t STAGE_FAILED =
    5u;

  // pointer types
  using RawPtr =
    common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__M7xxFirmwareUpgradeProgress
    std::shared_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__M7xxFirmwareUpgradeProgress
    std::shared_ptr<common_interface::msg::M7xxFirmwareUpgradeProgress_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const M7xxFirmwareUpgradeProgress_ & other) const
  {
    if (this->percent != other.percent) {
      return false;
    }
    if (this->stage != other.stage) {
      return false;
    }
    if (this->result_code != other.result_code) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const M7xxFirmwareUpgradeProgress_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct M7xxFirmwareUpgradeProgress_

// alias to use template instance with default allocator
using M7xxFirmwareUpgradeProgress =
  common_interface::msg::M7xxFirmwareUpgradeProgress_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M7xxFirmwareUpgradeProgress_<ContainerAllocator>::STAGE_PREPARING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M7xxFirmwareUpgradeProgress_<ContainerAllocator>::STAGE_TRANSFERRING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M7xxFirmwareUpgradeProgress_<ContainerAllocator>::STAGE_RETRY_WAIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M7xxFirmwareUpgradeProgress_<ContainerAllocator>::STAGE_RECOVERING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M7xxFirmwareUpgradeProgress_<ContainerAllocator>::STAGE_SUCCEEDED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M7xxFirmwareUpgradeProgress_<ContainerAllocator>::STAGE_FAILED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__M7XX_FIRMWARE_UPGRADE_PROGRESS__STRUCT_HPP_
