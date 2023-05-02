#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROS2_MOTOR_CONTROL_DEMO_EXPORT __attribute__((dllexport))
#define ROS2_MOTOR_CONTROL_DEMO_IMPORT __attribute__((dllimport))
#else
#define ROS2_MOTOR_CONTROL_DEMO_EXPORT __declspec(dllexport)
#define ROS2_MOTOR_CONTROL_DEMO_IMPORT __declspec(dllimport)
#endif
#ifdef ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL
#define ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC ROS2_MOTOR_CONTROL_DEMO_EXPORT
#else
#define ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC ROS2_MOTOR_CONTROL_DEMO_IMPORT
#endif
#define ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC_TYPE ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
#define ROS2_CONTROL_DEMO_EXAMPLE_2_LOCAL
#else
#define ROS2_MOTOR_CONTROL_DEMO_EXPORT __attribute__((visibility("default")))
#define ROS2_MOTOR_CONTROL_DEMO_IMPORT
#if __GNUC__ >= 4
#define ROS2_MOTOR_CONTROL_DEMO_PUBLIC __attribute__((visibility("default")))
#define ROS2_MOTOR_CONTROL_DEMO_LOCAL __attribute__((visibility("hidden")))
#else
#define ROS2_MOTOR_CONTROL_DEMO_PUBLIC
#define ROS2_MOTOR_CONTROL_DEMO_LOCAL
#endif
#define ROS2_MOTOR_CONTROL_DEMO_PUBLIC_TYPE
#endif