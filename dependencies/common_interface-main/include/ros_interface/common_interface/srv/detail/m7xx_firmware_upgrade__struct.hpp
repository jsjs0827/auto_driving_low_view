// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/M7xxFirmwareUpgrade.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__M7XX_FIRMWARE_UPGRADE__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__M7XX_FIRMWARE_UPGRADE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct M7xxFirmwareUpgrade_Request_
{
  using Type = M7xxFirmwareUpgrade_Request_<ContainerAllocator>;

  explicit M7xxFirmwareUpgrade_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->package_path = "";
    }
  }

  explicit M7xxFirmwareUpgrade_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : package_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->package_path = "";
    }
  }

  // field types and members
  using _package_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _package_path_type package_path;

  // setters for named parameter idiom
  Type & set__package_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->package_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Request
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Request
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const M7xxFirmwareUpgrade_Request_ & other) const
  {
    if (this->package_path != other.package_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const M7xxFirmwareUpgrade_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct M7xxFirmwareUpgrade_Request_

// alias to use template instance with default allocator
using M7xxFirmwareUpgrade_Request =
  common_interface::srv::M7xxFirmwareUpgrade_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct M7xxFirmwareUpgrade_Response_
{
  using Type = M7xxFirmwareUpgrade_Response_<ContainerAllocator>;

  explicit M7xxFirmwareUpgrade_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->message = "";
    }
  }

  explicit M7xxFirmwareUpgrade_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->message = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Response
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__M7xxFirmwareUpgrade_Response
    std::shared_ptr<common_interface::srv::M7xxFirmwareUpgrade_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const M7xxFirmwareUpgrade_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const M7xxFirmwareUpgrade_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct M7xxFirmwareUpgrade_Response_

// alias to use template instance with default allocator
using M7xxFirmwareUpgrade_Response =
  common_interface::srv::M7xxFirmwareUpgrade_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct M7xxFirmwareUpgrade
{
  using Request = common_interface::srv::M7xxFirmwareUpgrade_Request;
  using Response = common_interface::srv::M7xxFirmwareUpgrade_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__M7XX_FIRMWARE_UPGRADE__STRUCT_HPP_
