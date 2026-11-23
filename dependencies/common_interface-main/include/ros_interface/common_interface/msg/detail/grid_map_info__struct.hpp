// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/GridMapInfo.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_interface__msg__GridMapInfo __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__GridMapInfo __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GridMapInfo_
{
  using Type = GridMapInfo_<ContainerAllocator>;

  explicit GridMapInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rows = 0;
      this->cols = 0;
      this->resolution_mm = 0;
    }
  }

  explicit GridMapInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rows = 0;
      this->cols = 0;
      this->resolution_mm = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rows_type =
    uint16_t;
  _rows_type rows;
  using _cols_type =
    uint16_t;
  _cols_type cols;
  using _resolution_mm_type =
    uint16_t;
  _resolution_mm_type resolution_mm;
  using _avg_h_cm_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _avg_h_cm_type avg_h_cm;
  using _min_h_cm_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _min_h_cm_type min_h_cm;
  using _max_h_cm_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _max_h_cm_type max_h_cm;
  using _type_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _type_type type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rows(
    const uint16_t & _arg)
  {
    this->rows = _arg;
    return *this;
  }
  Type & set__cols(
    const uint16_t & _arg)
  {
    this->cols = _arg;
    return *this;
  }
  Type & set__resolution_mm(
    const uint16_t & _arg)
  {
    this->resolution_mm = _arg;
    return *this;
  }
  Type & set__avg_h_cm(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->avg_h_cm = _arg;
    return *this;
  }
  Type & set__min_h_cm(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->min_h_cm = _arg;
    return *this;
  }
  Type & set__max_h_cm(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->max_h_cm = _arg;
    return *this;
  }
  Type & set__type(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::GridMapInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::GridMapInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::GridMapInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::GridMapInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__GridMapInfo
    std::shared_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__GridMapInfo
    std::shared_ptr<common_interface::msg::GridMapInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GridMapInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rows != other.rows) {
      return false;
    }
    if (this->cols != other.cols) {
      return false;
    }
    if (this->resolution_mm != other.resolution_mm) {
      return false;
    }
    if (this->avg_h_cm != other.avg_h_cm) {
      return false;
    }
    if (this->min_h_cm != other.min_h_cm) {
      return false;
    }
    if (this->max_h_cm != other.max_h_cm) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const GridMapInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GridMapInfo_

// alias to use template instance with default allocator
using GridMapInfo =
  common_interface::msg::GridMapInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__STRUCT_HPP_
