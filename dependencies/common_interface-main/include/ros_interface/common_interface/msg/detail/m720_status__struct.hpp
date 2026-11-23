// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/M720Status.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__M720Status __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__M720Status __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct M720Status_
{
  using Type = M720Status_<ContainerAllocator>;

  explicit M720Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->init_status = 0;
      this->fusion_status = 0;
      this->sensor_status = 0;
      this->gnss_status = 0;
      this->sat_num = 0;
    }
  }

  explicit M720Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->init_status = 0;
      this->fusion_status = 0;
      this->sensor_status = 0;
      this->gnss_status = 0;
      this->sat_num = 0;
    }
  }

  // field types and members
  using _init_status_type =
    uint8_t;
  _init_status_type init_status;
  using _fusion_status_type =
    uint8_t;
  _fusion_status_type fusion_status;
  using _sensor_status_type =
    uint8_t;
  _sensor_status_type sensor_status;
  using _gnss_status_type =
    uint8_t;
  _gnss_status_type gnss_status;
  using _sat_num_type =
    uint8_t;
  _sat_num_type sat_num;

  // setters for named parameter idiom
  Type & set__init_status(
    const uint8_t & _arg)
  {
    this->init_status = _arg;
    return *this;
  }
  Type & set__fusion_status(
    const uint8_t & _arg)
  {
    this->fusion_status = _arg;
    return *this;
  }
  Type & set__sensor_status(
    const uint8_t & _arg)
  {
    this->sensor_status = _arg;
    return *this;
  }
  Type & set__gnss_status(
    const uint8_t & _arg)
  {
    this->gnss_status = _arg;
    return *this;
  }
  Type & set__sat_num(
    const uint8_t & _arg)
  {
    this->sat_num = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t INIT_NONE =
    0u;
  static constexpr uint8_t INIT_RTK_NOT_FIXED =
    1u;
  static constexpr uint8_t INIT_RTK_FIXED_READY =
    2u;
  static constexpr uint8_t INIT_INITIALIZING =
    3u;
  static constexpr uint8_t INIT_FINISHED =
    4u;
  static constexpr uint8_t FUSION_INVALID =
    0u;
  static constexpr uint8_t FUSION_DEGRADED =
    1u;
  static constexpr uint8_t FUSION_NORMAL =
    2u;
  static constexpr uint8_t FUSION_GOOD =
    3u;
  static constexpr uint8_t SENSOR_GNSS_ERROR =
    1u;
  static constexpr uint8_t SENSOR_VISION_ERROR =
    2u;
  static constexpr uint8_t SENSOR_ODOMETER_ERROR =
    4u;
  static constexpr uint8_t SENSOR_M720_IMU_ERROR =
    8u;
  static constexpr uint8_t SENSOR_S3162_IMU_ERROR =
    16u;
  static constexpr uint8_t GNSS_NONE =
    0u;
  static constexpr uint8_t GNSS_NORAML =
    1u;
  static constexpr uint8_t GNSS_GODD =
    2u;

  // pointer types
  using RawPtr =
    common_interface::msg::M720Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::M720Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::M720Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::M720Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::M720Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::M720Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::M720Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::M720Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::M720Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::M720Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__M720Status
    std::shared_ptr<common_interface::msg::M720Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__M720Status
    std::shared_ptr<common_interface::msg::M720Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const M720Status_ & other) const
  {
    if (this->init_status != other.init_status) {
      return false;
    }
    if (this->fusion_status != other.fusion_status) {
      return false;
    }
    if (this->sensor_status != other.sensor_status) {
      return false;
    }
    if (this->gnss_status != other.gnss_status) {
      return false;
    }
    if (this->sat_num != other.sat_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const M720Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct M720Status_

// alias to use template instance with default allocator
using M720Status =
  common_interface::msg::M720Status_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::INIT_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::INIT_RTK_NOT_FIXED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::INIT_RTK_FIXED_READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::INIT_INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::INIT_FINISHED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::FUSION_INVALID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::FUSION_DEGRADED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::FUSION_NORMAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::FUSION_GOOD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::SENSOR_GNSS_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::SENSOR_VISION_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::SENSOR_ODOMETER_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::SENSOR_M720_IMU_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::SENSOR_S3162_IMU_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::GNSS_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::GNSS_NORAML;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t M720Status_<ContainerAllocator>::GNSS_GODD;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__STRUCT_HPP_
