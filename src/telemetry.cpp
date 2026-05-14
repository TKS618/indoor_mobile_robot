#include "telemetry.hpp"
#include "config.hpp"

#include <math.h>

Telemetry* Telemetry::instance = nullptr;

Telemetry::Telemetry()
: cmd_v(0.0f),
  cmd_w(0.0f),
  target_right_rad_per_sec(0.0f),
  target_left_rad_per_sec(0.0f),
  last_cmd_vel_time(0)
{
    instance = this;
}

bool Telemetry::begin()
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(
            &node,
            "teensy_base_controller",
            "",
            &support) != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &odom_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/odom") != RCL_RET_OK) {
        return false;
    }

    if (rclc_subscription_init_default(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &cmd_vel_sub,
            &cmd_vel_msg,
            &Telemetry::cmdVelCallback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    nav_msgs__msg__Odometry__init(&odom_msg);

    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

    last_cmd_vel_time = millis();

    return true;
}

void Telemetry::spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

void Telemetry::cmdVelCallback(const void* msg)
{
    const geometry_msgs__msg__Twist* received_msg =
        static_cast<const geometry_msgs__msg__Twist*>(msg);

    if (instance != nullptr) {
        instance->setCmdVel(
            received_msg->linear.x,
            received_msg->angular.z
        );
    }
}

void Telemetry::setCmdVel(float linear_x, float angular_z)
{
    cmd_v = linear_x;
    cmd_w = angular_z;
    last_cmd_vel_time = millis();

    calcWheelTarget();
}

void Telemetry::updateCmdVelTimeout()
{
    if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
        cmd_v = 0.0f;
        cmd_w = 0.0f;
        calcWheelTarget();
    }
}

void Telemetry::calcWheelTarget()
{
    target_right_rad_per_sec =
        (cmd_v + cmd_w * WHEEL_BASE * 0.5f) * WHEEL_RADIUS_INV;

    target_left_rad_per_sec =
        (cmd_v - cmd_w * WHEEL_BASE * 0.5f) * WHEEL_RADIUS_INV;
}

float Telemetry::getTargetRightRadPerSec() const
{
    return target_right_rad_per_sec;
}

float Telemetry::getTargetLeftRadPerSec() const
{
    return target_left_rad_per_sec;
}

void Telemetry::publishOdom(float x, float y, float theta)
{
    unsigned long now = millis();

    odom_msg.header.stamp.sec = now / 1000;
    odom_msg.header.stamp.nanosec = (now % 1000) * 1000000;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0f;

    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(theta * 0.5f);

    rcl_publish(&odom_pub, &odom_msg, NULL);
}