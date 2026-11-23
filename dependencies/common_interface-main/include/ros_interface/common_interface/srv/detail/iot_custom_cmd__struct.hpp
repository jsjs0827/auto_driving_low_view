// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/IotCustomCmd.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__IotCustomCmd_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__IotCustomCmd_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IotCustomCmd_Request_
{
  using Type = IotCustomCmd_Request_<ContainerAllocator>;

  explicit IotCustomCmd_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  explicit IotCustomCmd_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  // field types and members
  using _request_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::IotCustomCmd_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::IotCustomCmd_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::IotCustomCmd_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::IotCustomCmd_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__IotCustomCmd_Request
    std::shared_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__IotCustomCmd_Request
    std::shared_ptr<common_interface::srv::IotCustomCmd_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IotCustomCmd_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const IotCustomCmd_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IotCustomCmd_Request_

// alias to use template instance with default allocator
using IotCustomCmd_Request =
  common_interface::srv::IotCustomCmd_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__IotCustomCmd_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__IotCustomCmd_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IotCustomCmd_Response_
{
  using Type = IotCustomCmd_Response_<ContainerAllocator>;

  explicit IotCustomCmd_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reply = "";
    }
  }

  explicit IotCustomCmd_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reply(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reply = "";
    }
  }

  // field types and members
  using _reply_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reply_type reply;

  // setters for named parameter idiom
  Type & set__reply(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reply = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::IotCustomCmd_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::IotCustomCmd_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::IotCustomCmd_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::IotCustomCmd_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__IotCustomCmd_Response
    std::shared_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__IotCustomCmd_Response
    std::shared_ptr<common_interface::srv::IotCustomCmd_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IotCustomCmd_Response_ & other) const
  {
    if (this->reply != other.reply) {
      return false;
    }
    return true;
  }
  bool operator!=(const IotCustomCmd_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IotCustomCmd_Response_

// alias to use template instance with default allocator
using IotCustomCmd_Response =
  common_interface::srv::IotCustomCmd_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct IotCustomCmd
{
  using Request = common_interface::srv::IotCustomCmd_Request;
  using Response = common_interface::srv::IotCustomCmd_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__STRUCT_HPP_
