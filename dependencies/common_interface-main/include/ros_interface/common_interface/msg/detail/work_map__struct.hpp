// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/WorkMap.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'zones'
#include "common_interface/msg/detail/work_map_zone__struct.hpp"
// Member 'pathways'
#include "common_interface/msg/detail/path_way__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_interface__msg__WorkMap __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__WorkMap __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorkMap_
{
  using Type = WorkMap_<ContainerAllocator>;

  explicit WorkMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_id = 0ul;
      this->map_name = "";
      this->frame_id = "";
      this->active = false;
    }
  }

  explicit WorkMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_name(_alloc),
    frame_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_id = 0ul;
      this->map_name = "";
      this->frame_id = "";
      this->active = false;
    }
  }

  // field types and members
  using _map_id_type =
    uint32_t;
  _map_id_type map_id;
  using _map_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_name_type map_name;
  using _frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_id_type frame_id;
  using _active_type =
    bool;
  _active_type active;
  using _zones_type =
    std::vector<common_interface::msg::WorkMapZone_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapZone_<ContainerAllocator>>>;
  _zones_type zones;
  using _pathways_type =
    std::vector<common_interface::msg::PathWay_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::PathWay_<ContainerAllocator>>>;
  _pathways_type pathways;

  // setters for named parameter idiom
  Type & set__map_id(
    const uint32_t & _arg)
  {
    this->map_id = _arg;
    return *this;
  }
  Type & set__map_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_name = _arg;
    return *this;
  }
  Type & set__frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }
  Type & set__zones(
    const std::vector<common_interface::msg::WorkMapZone_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapZone_<ContainerAllocator>>> & _arg)
  {
    this->zones = _arg;
    return *this;
  }
  Type & set__pathways(
    const std::vector<common_interface::msg::PathWay_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::PathWay_<ContainerAllocator>>> & _arg)
  {
    this->pathways = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::WorkMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::WorkMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::WorkMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::WorkMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::WorkMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::WorkMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__WorkMap
    std::shared_ptr<common_interface::msg::WorkMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__WorkMap
    std::shared_ptr<common_interface::msg::WorkMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorkMap_ & other) const
  {
    if (this->map_id != other.map_id) {
      return false;
    }
    if (this->map_name != other.map_name) {
      return false;
    }
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    if (this->zones != other.zones) {
      return false;
    }
    if (this->pathways != other.pathways) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorkMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorkMap_

// alias to use template instance with default allocator
using WorkMap =
  common_interface::msg::WorkMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__STRUCT_HPP_
