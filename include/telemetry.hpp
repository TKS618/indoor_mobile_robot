#pragma once

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <rosidl_runtime_c/string_functions.h>

class Telemetry {
public:
    Telemetry();

    bool begin();
    void spin();

    void updateCmdVelTimeout();

    float getTargetRightRadPerSec() const;
    float getTargetLeftRadPerSec() const;

    void publishOdom(float x, float y, float theta);

private:
    static Telemetry* instance;
    static void cmdVelCallback(const void* msg);

    void setCmdVel(float linear_x, float angular_z);
    void calcWheelTarget();

private:
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;

    rcl_publisher_t odom_pub;
    rcl_subscription_t cmd_vel_sub;
    rclc_executor_t executor;

    geometry_msgs__msg__Twist cmd_vel_msg;
    nav_msgs__msg__Odometry odom_msg;

    float cmd_v;
    float cmd_w;

    float target_right_rad_per_sec;
    float target_left_rad_per_sec;

    unsigned long last_cmd_vel_time;
};