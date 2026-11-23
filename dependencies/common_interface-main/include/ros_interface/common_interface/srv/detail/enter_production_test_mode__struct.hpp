// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/EnterProductionTestMode.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__ENTER_PRODUCTION_TEST_MODE__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__ENTER_PRODUCTION_TEST_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__EnterProductionTestMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__EnterProductionTestMode_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnterProductionTestMode_Request_
{
  using Type = EnterProductionTestMode_Request_<ContainerAllocator>;

  explicit EnterProductionTestMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->expected_version = "";
    }
  }

  explicit EnterProductionTestMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : expected_version(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->expected_version = "";
    }
  }

  // field types and members
  using _expected_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _expected_version_type expected_version;

  // setters for named parameter idiom
  Type & set__expected_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->expected_version = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__EnterProductionTestMode_Request
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__EnterProductionTestMode_Request
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnterProductionTestMode_Request_ & other) const
  {
    if (this->expected_version != other.expected_version) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnterProductionTestMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnterProductionTestMode_Request_

// alias to use template instance with default allocator
using EnterProductionTestMode_Request =
  common_interface::srv::EnterProductionTestMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__EnterProductionTestMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__EnterProductionTestMode_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnterProductionTestMode_Response_
{
  using Type = EnterProductionTestMode_Response_<ContainerAllocator>;

  explicit EnterProductionTestMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_code = 0l;
    }
  }

  explicit EnterProductionTestMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_code = 0l;
    }
  }

  // field types and members
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _error_codes_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _error_codes_type error_codes;
  using _error_details_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _error_details_type error_details;

  // setters for named parameter idiom
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__error_codes(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->error_codes = _arg;
    return *this;
  }
  Type & set__error_details(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->error_details = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__EnterProductionTestMode_Response
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__EnterProductionTestMode_Response
    std::shared_ptr<common_interface::srv::EnterProductionTestMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnterProductionTestMode_Response_ & other) const
  {
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->error_codes != other.error_codes) {
      return false;
    }
    if (this->error_details != other.error_details) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnterProductionTestMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnterProductionTestMode_Response_

// alias to use template instance with default allocator
using EnterProductionTestMode_Response =
  common_interface::srv::EnterProductionTestMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct EnterProductionTestMode
{
  using Request = common_interface::srv::EnterProductionTestMode_Request;
  using Response = common_interface::srv::EnterProductionTestMode_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__ENTER_PRODUCTION_TEST_MODE__STRUCT_HPP_
