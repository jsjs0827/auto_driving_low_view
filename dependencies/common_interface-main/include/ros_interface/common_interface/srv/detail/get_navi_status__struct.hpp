// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/GetNaviStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__GET_NAVI_STATUS__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__GET_NAVI_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__GetNaviStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__GetNaviStatus_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetNaviStatus_Request_
{
  using Type = GetNaviStatus_Request_<ContainerAllocator>;

  explicit GetNaviStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0l;
    }
  }

  explicit GetNaviStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0l;
    }
  }

  // field types and members
  using _cmd_type =
    int32_t;
  _cmd_type cmd;

  // setters for named parameter idiom
  Type & set__cmd(
    const int32_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::GetNaviStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::GetNaviStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetNaviStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetNaviStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__GetNaviStatus_Request
    std::shared_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__GetNaviStatus_Request
    std::shared_ptr<common_interface::srv::GetNaviStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetNaviStatus_Request_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetNaviStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetNaviStatus_Request_

// alias to use template instance with default allocator
using GetNaviStatus_Request =
  common_interface::srv::GetNaviStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__GetNaviStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__GetNaviStatus_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetNaviStatus_Response_
{
  using Type = GetNaviStatus_Response_<ContainerAllocator>;

  explicit GetNaviStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
    }
  }

  explicit GetNaviStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    common_interface::srv::GetNaviStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::GetNaviStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetNaviStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetNaviStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__GetNaviStatus_Response
    std::shared_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__GetNaviStatus_Response
    std::shared_ptr<common_interface::srv::GetNaviStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetNaviStatus_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetNaviStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetNaviStatus_Response_

// alias to use template instance with default allocator
using GetNaviStatus_Response =
  common_interface::srv::GetNaviStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct GetNaviStatus
{
  using Request = common_interface::srv::GetNaviStatus_Request;
  using Response = common_interface::srv::GetNaviStatus_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__GET_NAVI_STATUS__STRUCT_HPP_
