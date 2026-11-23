// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/PathWay.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'polygon'
#include "common_interface/msg/detail/work_map_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_interface__msg__PathWay __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__PathWay __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PathWay_
{
  using Type = PathWay_<ContainerAllocator>;

  explicit PathWay_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pathway_id = 0ul;
      this->pathway_name = "";
      this->parent_id = 0ul;
      this->from_zone_id = 0ul;
      this->to_zone_id = 0ul;
      this->pathway_width = 0.0;
      this->description = "";
    }
  }

  explicit PathWay_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pathway_name(_alloc),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pathway_id = 0ul;
      this->pathway_name = "";
      this->parent_id = 0ul;
      this->from_zone_id = 0ul;
      this->to_zone_id = 0ul;
      this->pathway_width = 0.0;
      this->description = "";
    }
  }

  // field types and members
  using _pathway_id_type =
    uint32_t;
  _pathway_id_type pathway_id;
  using _pathway_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pathway_name_type pathway_name;
  using _parent_id_type =
    uint32_t;
  _parent_id_type parent_id;
  using _from_zone_id_type =
    uint32_t;
  _from_zone_id_type from_zone_id;
  using _to_zone_id_type =
    uint32_t;
  _to_zone_id_type to_zone_id;
  using _polygon_type =
    std::vector<common_interface::msg::WorkMapPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapPoint_<ContainerAllocator>>>;
  _polygon_type polygon;
  using _pathway_width_type =
    double;
  _pathway_width_type pathway_width;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__pathway_id(
    const uint32_t & _arg)
  {
    this->pathway_id = _arg;
    return *this;
  }
  Type & set__pathway_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pathway_name = _arg;
    return *this;
  }
  Type & set__parent_id(
    const uint32_t & _arg)
  {
    this->parent_id = _arg;
    return *this;
  }
  Type & set__from_zone_id(
    const uint32_t & _arg)
  {
    this->from_zone_id = _arg;
    return *this;
  }
  Type & set__to_zone_id(
    const uint32_t & _arg)
  {
    this->to_zone_id = _arg;
    return *this;
  }
  Type & set__polygon(
    const std::vector<common_interface::msg::WorkMapPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapPoint_<ContainerAllocator>>> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }
  Type & set__pathway_width(
    const double & _arg)
  {
    this->pathway_width = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::PathWay_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::PathWay_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::PathWay_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::PathWay_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::PathWay_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::PathWay_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::PathWay_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::PathWay_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::PathWay_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::PathWay_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__PathWay
    std::shared_ptr<common_interface::msg::PathWay_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__PathWay
    std::shared_ptr<common_interface::msg::PathWay_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathWay_ & other) const
  {
    if (this->pathway_id != other.pathway_id) {
      return false;
    }
    if (this->pathway_name != other.pathway_name) {
      return false;
    }
    if (this->parent_id != other.parent_id) {
      return false;
    }
    if (this->from_zone_id != other.from_zone_id) {
      return false;
    }
    if (this->to_zone_id != other.to_zone_id) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    if (this->pathway_width != other.pathway_width) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathWay_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathWay_

// alias to use template instance with default allocator
using PathWay =
  common_interface::msg::PathWay_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__STRUCT_HPP_
