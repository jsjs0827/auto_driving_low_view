// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/WorkMapZone.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'elements'
#include "common_interface/msg/detail/work_map_element__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_interface__msg__WorkMapZone __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__WorkMapZone __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorkMapZone_
{
  using Type = WorkMapZone_<ContainerAllocator>;

  explicit WorkMapZone_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->zone_id = 0ul;
      this->zone_name = "";
      this->parent_id = 0ul;
    }
  }

  explicit WorkMapZone_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : zone_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->zone_id = 0ul;
      this->zone_name = "";
      this->parent_id = 0ul;
    }
  }

  // field types and members
  using _zone_id_type =
    uint32_t;
  _zone_id_type zone_id;
  using _zone_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _zone_name_type zone_name;
  using _parent_id_type =
    uint32_t;
  _parent_id_type parent_id;
  using _elements_type =
    std::vector<common_interface::msg::WorkMapElement_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapElement_<ContainerAllocator>>>;
  _elements_type elements;

  // setters for named parameter idiom
  Type & set__zone_id(
    const uint32_t & _arg)
  {
    this->zone_id = _arg;
    return *this;
  }
  Type & set__zone_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->zone_name = _arg;
    return *this;
  }
  Type & set__parent_id(
    const uint32_t & _arg)
  {
    this->parent_id = _arg;
    return *this;
  }
  Type & set__elements(
    const std::vector<common_interface::msg::WorkMapElement_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapElement_<ContainerAllocator>>> & _arg)
  {
    this->elements = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::WorkMapZone_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::WorkMapZone_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMapZone_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMapZone_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__WorkMapZone
    std::shared_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__WorkMapZone
    std::shared_ptr<common_interface::msg::WorkMapZone_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorkMapZone_ & other) const
  {
    if (this->zone_id != other.zone_id) {
      return false;
    }
    if (this->zone_name != other.zone_name) {
      return false;
    }
    if (this->parent_id != other.parent_id) {
      return false;
    }
    if (this->elements != other.elements) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorkMapZone_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorkMapZone_

// alias to use template instance with default allocator
using WorkMapZone =
  common_interface::msg::WorkMapZone_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__STRUCT_HPP_
