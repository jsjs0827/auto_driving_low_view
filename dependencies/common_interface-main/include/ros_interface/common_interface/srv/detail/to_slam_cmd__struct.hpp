// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/ToSlamCmd.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__TO_SLAM_CMD__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__TO_SLAM_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__ToSlamCmd_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__ToSlamCmd_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ToSlamCmd_Request_
{
  using Type = ToSlamCmd_Request_<ContainerAllocator>;

  explicit ToSlamCmd_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0l;
      this->json_data = "";
    }
  }

  explicit ToSlamCmd_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : json_data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0l;
      this->json_data = "";
    }
  }

  // field types and members
  using _cmd_type =
    int32_t;
  _cmd_type cmd;
  using _json_data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _json_data_type json_data;

  // setters for named parameter idiom
  Type & set__cmd(
    const int32_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__json_data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->json_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::ToSlamCmd_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::ToSlamCmd_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::ToSlamCmd_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::ToSlamCmd_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__ToSlamCmd_Request
    std::shared_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__ToSlamCmd_Request
    std::shared_ptr<common_interface::srv::ToSlamCmd_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToSlamCmd_Request_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->json_data != other.json_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToSlamCmd_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToSlamCmd_Request_

// alias to use template instance with default allocator
using ToSlamCmd_Request =
  common_interface::srv::ToSlamCmd_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__ToSlamCmd_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__ToSlamCmd_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ToSlamCmd_Response_
{
  using Type = ToSlamCmd_Response_<ContainerAllocator>;

  explicit ToSlamCmd_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
    }
  }

  explicit ToSlamCmd_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
    }
  }

  // field types and members
  using _result_type =
    int32_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const int32_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::ToSlamCmd_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::ToSlamCmd_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::ToSlamCmd_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::ToSlamCmd_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__ToSlamCmd_Response
    std::shared_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__ToSlamCmd_Response
    std::shared_ptr<common_interface::srv::ToSlamCmd_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToSlamCmd_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToSlamCmd_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToSlamCmd_Response_

// alias to use template instance with default allocator
using ToSlamCmd_Response =
  common_interface::srv::ToSlamCmd_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct ToSlamCmd
{
  using Request = common_interface::srv::ToSlamCmd_Request;
  using Response = common_interface::srv::ToSlamCmd_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__TO_SLAM_CMD__STRUCT_HPP_
