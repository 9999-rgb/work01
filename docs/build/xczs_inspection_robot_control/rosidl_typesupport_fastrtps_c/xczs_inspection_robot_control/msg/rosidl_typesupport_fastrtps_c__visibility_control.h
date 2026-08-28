// generated from
// rosidl_typesupport_fastrtps_c/resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
#define XCZS_INSPECTION_ROBOT_CONTROL__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_xczs_inspection_robot_control __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xczs_inspection_robot_control __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_xczs_inspection_robot_control __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xczs_inspection_robot_control __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_C_BUILDING_DLL_xczs_inspection_robot_control
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_control ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_xczs_inspection_robot_control
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_control ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xczs_inspection_robot_control
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_xczs_inspection_robot_control __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xczs_inspection_robot_control
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_control __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_control
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
