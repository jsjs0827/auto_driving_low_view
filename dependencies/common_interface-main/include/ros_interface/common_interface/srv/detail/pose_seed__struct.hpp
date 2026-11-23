// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/PoseSeed.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__PoseSeed_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__PoseSeed_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PoseSeed_Request_
{
  using Type = PoseSeed_Request_<ContainerAllocator>;

  explicit PoseSeed_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->option = 0;
      this->has_pose_seed = false;
      std::fill<typename std::array<double, 21>::iterator, double>(this->seed.begin(), this->seed.end(), 0.0);
    }
  }

  explicit PoseSeed_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : seed(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->option = 0;
      this->has_pose_seed = false;
      std::fill<typename std::array<double, 21>::iterator, double>(this->seed.begin(), this->seed.end(), 0.0);
    }
  }

  // field types and members
  using _option_type =
    uint8_t;
  _option_type option;
  using _has_pose_seed_type =
    bool;
  _has_pose_seed_type has_pose_seed;
  using _seed_type =
    std::array<double, 21>;
  _seed_type seed;

  // setters for named parameter idiom
  Type & set__option(
    const uint8_t & _arg)
  {
    this->option = _arg;
    return *this;
  }
  Type & set__has_pose_seed(
    const bool & _arg)
  {
    this->has_pose_seed = _arg;
    return *this;
  }
  Type & set__seed(
    const std::array<double, 21> & _arg)
  {
    this->seed = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t OPTION_MEMORIZE =
    1u;
  static constexpr uint8_t OPTION_INJECT =
    2u;

  // pointer types
  using RawPtr =
    common_interface::srv::PoseSeed_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::PoseSeed_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::PoseSeed_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::PoseSeed_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__PoseSeed_Request
    std::shared_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__PoseSeed_Request
    std::shared_ptr<common_interface::srv::PoseSeed_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseSeed_Request_ & other) const
  {
    if (this->option != other.option) {
      return false;
    }
    if (this->has_pose_seed != other.has_pose_seed) {
      return false;
    }
    if (this->seed != other.seed) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseSeed_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseSeed_Request_

// alias to use template instance with default allocator
using PoseSeed_Request =
  common_interface::srv::PoseSeed_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PoseSeed_Request_<ContainerAllocator>::OPTION_MEMORIZE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PoseSeed_Request_<ContainerAllocator>::OPTION_INJECT;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__PoseSeed_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__PoseSeed_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PoseSeed_Response_
{
  using Type = PoseSeed_Response_<ContainerAllocator>;

  explicit PoseSeed_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->error_code = 0;
      this->message = "";
      std::fill<typename std::array<double, 21>::iterator, double>(this->seed.begin(), this->seed.end(), 0.0);
    }
  }

  explicit PoseSeed_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    seed(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->error_code = 0;
      this->message = "";
      std::fill<typename std::array<double, 21>::iterator, double>(this->seed.begin(), this->seed.end(), 0.0);
    }
  }

  // field types and members
  using _result_type =
    uint8_t;
  _result_type result;
  using _error_code_type =
    uint16_t;
  _error_code_type error_code;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _seed_type =
    std::array<double, 21>;
  _seed_type seed;

  // setters for named parameter idiom
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint16_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__seed(
    const std::array<double, 21> & _arg)
  {
    this->seed = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t RESULT_REJECTED =
    0u;
  static constexpr uint8_t RESULT_ACCEPTED =
    1u;

  // pointer types
  using RawPtr =
    common_interface::srv::PoseSeed_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::PoseSeed_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::PoseSeed_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::PoseSeed_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__PoseSeed_Response
    std::shared_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__PoseSeed_Response
    std::shared_ptr<common_interface::srv::PoseSeed_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseSeed_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->seed != other.seed) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseSeed_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseSeed_Response_

// alias to use template instance with default allocator
using PoseSeed_Response =
  common_interface::srv::PoseSeed_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PoseSeed_Response_<ContainerAllocator>::RESULT_REJECTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PoseSeed_Response_<ContainerAllocator>::RESULT_ACCEPTED;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct PoseSeed
{
  using Request = common_interface::srv::PoseSeed_Request;
  using Response = common_interface::srv::PoseSeed_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__STRUCT_HPP_
