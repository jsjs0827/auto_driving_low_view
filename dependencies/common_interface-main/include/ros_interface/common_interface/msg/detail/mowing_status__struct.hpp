// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/MowingStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__MowingStatus __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__MowingStatus __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowingStatus_
{
  using Type = MowingStatus_<ContainerAllocator>;

  explicit MowingStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_mowing_area = 0.0f;
      this->mowed_area = 0.0f;
      this->mowing_progress = 0.0f;
    }
  }

  explicit MowingStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_mowing_area = 0.0f;
      this->mowed_area = 0.0f;
      this->mowing_progress = 0.0f;
    }
  }

  // field types and members
  using _total_mowing_area_type =
    float;
  _total_mowing_area_type total_mowing_area;
  using _mowed_area_type =
    float;
  _mowed_area_type mowed_area;
  using _mowing_progress_type =
    float;
  _mowing_progress_type mowing_progress;

  // setters for named parameter idiom
  Type & set__total_mowing_area(
    const float & _arg)
  {
    this->total_mowing_area = _arg;
    return *this;
  }
  Type & set__mowed_area(
    const float & _arg)
  {
    this->mowed_area = _arg;
    return *this;
  }
  Type & set__mowing_progress(
    const float & _arg)
  {
    this->mowing_progress = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::MowingStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::MowingStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::MowingStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::MowingStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MowingStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MowingStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::MowingStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::MowingStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::MowingStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::MowingStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__MowingStatus
    std::shared_ptr<common_interface::msg::MowingStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__MowingStatus
    std::shared_ptr<common_interface::msg::MowingStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowingStatus_ & other) const
  {
    if (this->total_mowing_area != other.total_mowing_area) {
      return false;
    }
    if (this->mowed_area != other.mowed_area) {
      return false;
    }
    if (this->mowing_progress != other.mowing_progress) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowingStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowingStatus_

// alias to use template instance with default allocator
using MowingStatus =
  common_interface::msg::MowingStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__STRUCT_HPP_
