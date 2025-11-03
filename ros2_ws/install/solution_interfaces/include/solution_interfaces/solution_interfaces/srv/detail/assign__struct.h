// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from solution_interfaces:srv/Assign.idl
// generated code does not contain a copyright notice

#ifndef SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__STRUCT_H_
#define SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'drone_name'
// Member 'role'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Assign in the package solution_interfaces.
typedef struct solution_interfaces__srv__Assign_Request
{
  rosidl_runtime_c__String drone_name;
  rosidl_runtime_c__String role;
} solution_interfaces__srv__Assign_Request;

// Struct for a sequence of solution_interfaces__srv__Assign_Request.
typedef struct solution_interfaces__srv__Assign_Request__Sequence
{
  solution_interfaces__srv__Assign_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} solution_interfaces__srv__Assign_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Assign in the package solution_interfaces.
typedef struct solution_interfaces__srv__Assign_Response
{
  bool success;
} solution_interfaces__srv__Assign_Response;

// Struct for a sequence of solution_interfaces__srv__Assign_Response.
typedef struct solution_interfaces__srv__Assign_Response__Sequence
{
  solution_interfaces__srv__Assign_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} solution_interfaces__srv__Assign_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__STRUCT_H_
