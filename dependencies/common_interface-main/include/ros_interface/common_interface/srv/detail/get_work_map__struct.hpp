// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/GetWorkMap.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__GET_WORK_MAP__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__GET_WORK_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__GetWorkMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__GetWorkMap_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetWorkMap_Request_
{
  using Type = GetWorkMap_Request_<ContainerAllocator>;

  explicit GetWorkMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0l;
      this->map_id = 0ul;
    }
  }

  explicit GetWorkMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0l;
      this->map_id = 0ul;
    }
  }

  // field types and members
  using _cmd_type =
    int32_t;
  _cmd_type cmd;
  using _map_id_type =
    uint32_t;
  _map_id_type map_id;

  // setters for named parameter idiom
  Type & set__cmd(
    const int32_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__map_id(
    const uint32_t & _arg)
  {
    this->map_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::GetWorkMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::GetWorkMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetWorkMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetWorkMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__GetWorkMap_Request
    std::shared_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__GetWorkMap_Request
    std::shared_ptr<common_interface::srv::GetWorkMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetWorkMap_Request_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->map_id != other.map_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetWorkMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetWorkMap_Request_

// alias to use template instance with default allocator
using GetWorkMap_Request =
  common_interface::srv::GetWorkMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface


// Include directives for member types
// Member 'maps'
#include "common_interface/msg/detail/work_map__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_interface__srv__GetWorkMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__GetWorkMap_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetWorkMap_Response_
{
  using Type = GetWorkMap_Response_<ContainerAllocator>;

  explicit GetWorkMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
    }
  }

  explicit GetWorkMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _maps_type =
    std::vector<common_interface::msg::WorkMap_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMap_<ContainerAllocator>>>;
  _maps_type maps;

  // setters for named parameter idiom
  Type & set__result(
    const int32_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__maps(
    const std::vector<common_interface::msg::WorkMap_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_interface::msg::WorkMap_<ContainerAllocator>>> & _arg)
  {
    this->maps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::GetWorkMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::GetWorkMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetWorkMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::GetWorkMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__GetWorkMap_Response
    std::shared_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__GetWorkMap_Response
    std::shared_ptr<common_interface::srv::GetWorkMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetWorkMap_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->maps != other.maps) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetWorkMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetWorkMap_Response_

// alias to use template instance with default allocator
using GetWorkMap_Response =
  common_interface::srv::GetWorkMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct GetWorkMap
{
  using Request = common_interface::srv::GetWorkMap_Request;
  using Response = common_interface::srv::GetWorkMap_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__GET_WORK_MAP__STRUCT_HPP_
