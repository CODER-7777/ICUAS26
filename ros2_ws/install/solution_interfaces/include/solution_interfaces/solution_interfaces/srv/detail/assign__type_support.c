// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from solution_interfaces:srv/Assign.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "solution_interfaces/srv/detail/assign__rosidl_typesupport_introspection_c.h"
#include "solution_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "solution_interfaces/srv/detail/assign__functions.h"
#include "solution_interfaces/srv/detail/assign__struct.h"


// Include directives for member types
// Member `drone_name`
// Member `role`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  solution_interfaces__srv__Assign_Request__init(message_memory);
}

void solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_fini_function(void * message_memory)
{
  solution_interfaces__srv__Assign_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_member_array[2] = {
  {
    "drone_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(solution_interfaces__srv__Assign_Request, drone_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "role",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(solution_interfaces__srv__Assign_Request, role),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_members = {
  "solution_interfaces__srv",  // message namespace
  "Assign_Request",  // message name
  2,  // number of fields
  sizeof(solution_interfaces__srv__Assign_Request),
  solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_member_array,  // message members
  solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_type_support_handle = {
  0,
  &solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_solution_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign_Request)() {
  if (!solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_type_support_handle.typesupport_identifier) {
    solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &solution_interfaces__srv__Assign_Request__rosidl_typesupport_introspection_c__Assign_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "solution_interfaces/srv/detail/assign__rosidl_typesupport_introspection_c.h"
// already included above
// #include "solution_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "solution_interfaces/srv/detail/assign__functions.h"
// already included above
// #include "solution_interfaces/srv/detail/assign__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  solution_interfaces__srv__Assign_Response__init(message_memory);
}

void solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_fini_function(void * message_memory)
{
  solution_interfaces__srv__Assign_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(solution_interfaces__srv__Assign_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_members = {
  "solution_interfaces__srv",  // message namespace
  "Assign_Response",  // message name
  1,  // number of fields
  sizeof(solution_interfaces__srv__Assign_Response),
  solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_member_array,  // message members
  solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_type_support_handle = {
  0,
  &solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_solution_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign_Response)() {
  if (!solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_type_support_handle.typesupport_identifier) {
    solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &solution_interfaces__srv__Assign_Response__rosidl_typesupport_introspection_c__Assign_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "solution_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "solution_interfaces/srv/detail/assign__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_members = {
  "solution_interfaces__srv",  // service namespace
  "Assign",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_Request_message_type_support_handle,
  NULL  // response message
  // solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_Response_message_type_support_handle
};

static rosidl_service_type_support_t solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_type_support_handle = {
  0,
  &solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_solution_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign)() {
  if (!solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_type_support_handle.typesupport_identifier) {
    solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, solution_interfaces, srv, Assign_Response)()->data;
  }

  return &solution_interfaces__srv__detail__assign__rosidl_typesupport_introspection_c__Assign_service_type_support_handle;
}
