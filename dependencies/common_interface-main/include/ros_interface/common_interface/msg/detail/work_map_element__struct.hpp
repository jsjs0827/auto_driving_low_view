// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/WorkMapElement.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__STRUCT_HPP_

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
# define DEPRECATED__common_interface__msg__WorkMapElement __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__WorkMapElement __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorkMapElement_
{
  using Type = WorkMapElement_<ContainerAllocator>;

  explicit WorkMapElement_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->element_id = 0ul;
      this->contour_type = 0ul;
    }
  }

  explicit WorkMapElement_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->element_id = 0ul;
      this->contour_type = 0ul;
    }
  }

  // field types and members
  using _element_id_type =
    uint32_t;
  _element_id_type element_id;
  using _contour_type_type =
    uint32_t;
  _contour_type_type contour_type;
  using _polygon_type =
    std::vector<common_interface::msg::WorkMapPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapPoint_<ContainerAllocator>>>;
  _polygon_type polygon;

  // setters for named parameter idiom
  Type & set__element_id(
    const uint32_t & _arg)
  {
    this->element_id = _arg;
    return *this;
  }
  Type & set__contour_type(
    const uint32_t & _arg)
  {
    this->contour_type = _arg;
    return *this;
  }
  Type & set__polygon(
    const std::vector<common_interface::msg::WorkMapPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMapPoint_<ContainerAllocator>>> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::WorkMapElement_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::WorkMapElement_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMapElement_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMapElement_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__WorkMapElement
    std::shared_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__WorkMapElement
    std::shared_ptr<common_interface::msg::WorkMapElement_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorkMapElement_ & other) const
  {
    if (this->element_id != other.element_id) {
      return false;
    }
    if (this->contour_type != other.contour_type) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorkMapElement_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorkMapElement_

// alias to use template instance with default allocator
using WorkMapElement =
  common_interface::msg::WorkMapElement_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__STRUCT_HPP_
