// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/DebugLight.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_LIGHT__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_LIGHT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__DebugLight __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__DebugLight __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DebugLight_
{
  using Type = DebugLight_<ContainerAllocator>;

  explicit DebugLight_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_x = 0l;
      this->radius = 0.0;
      this->is_light = false;
    }
  }

  explicit DebugLight_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_x = 0l;
      this->radius = 0.0;
      this->is_light = false;
    }
  }

  // field types and members
  using _center_x_type =
    int32_t;
  _center_x_type center_x;
  using _radius_type =
    double;
  _radius_type radius;
  using _is_light_type =
    bool;
  _is_light_type is_light;

  // setters for named parameter idiom
  Type & set__center_x(
    const int32_t & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__radius(
    const double & _arg)
  {
    this->radius = _arg;
    return *this;
  }
  Type & set__is_light(
    const bool & _arg)
  {
    this->is_light = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::DebugLight_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::DebugLight_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::DebugLight_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::DebugLight_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__DebugLight
    std::shared_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__DebugLight
    std::shared_ptr<auto_aim_interfaces::msg::DebugLight_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DebugLight_ & other) const
  {
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->radius != other.radius) {
      return false;
    }
    if (this->is_light != other.is_light) {
      return false;
    }
    return true;
  }
  bool operator!=(const DebugLight_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DebugLight_

// alias to use template instance with default allocator
using DebugLight =
  auto_aim_interfaces::msg::DebugLight_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_LIGHT__STRUCT_HPP_
