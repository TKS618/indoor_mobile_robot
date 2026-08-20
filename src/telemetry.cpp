#include "telemetry.hpp"
#include "config.hpp"

#include <math.h>

Telemetry* Telemetry::instance = nullptr;

Telemetry::Telemetry()
: state(ConnectionState::WAITING_AGENT),
  last_ping_time(0),
  allocator(rcl_get_default_allocator()),
  support{},
  node(rcl_get_zero_initialized_node()),
  odom_pub(rcl_get_zero_initialized_publisher()),
  cmd_vel_sub(rcl_get_zero_initialized_subscription()),
  executor(rclc_executor_get_zero_initialized_executor()),
  cmd_vel_msg{},
  odom_msg{},
  support_initialized(false),
  node_initialized(false),
  publisher_initialized(false),
  subscription_initialized(false),
  executor_initialized(false),
  odom_msg_initialized(false),
  cmd_v(0.0f),
  cmd_w(0.0f),
  target_right_rad_per_sec(0.0f),
  target_left_rad_per_sec(0.0f),
  last_cmd_vel_time(0)
{
    instance = this;
}

void Telemetry::update()
{
    const unsigned long now = millis();

    switch (state) {
    case ConnectionState::WAITING_AGENT:
        stopRobot();

        if (now - last_ping_time < PING_INTERVAL_MS) {
            break;
        }

        last_ping_time = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            state = ConnectionState::AGENT_AVAILABLE;
        }
        break;

    case ConnectionState::AGENT_AVAILABLE:
        if (createEntities()) {
            state = ConnectionState::AGENT_CONNECTED;
            last_ping_time = now;
        } else {
            destroyEntities();
            state = ConnectionState::WAITING_AGENT;
            last_ping_time = now;
        }
        break;

    case ConnectionState::AGENT_CONNECTED:
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        if (now - last_ping_time >= PING_INTERVAL_MS) {
            last_ping_time = now;
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                state = ConnectionState::AGENT_DISCONNECTED;
            }
        }
        break;

    case ConnectionState::AGENT_DISCONNECTED:
        stopRobot();
        destroyEntities();
        state = ConnectionState::WAITING_AGENT;
        last_ping_time = now;
        break;
    }
}

bool Telemetry::createEntities()
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    support_initialized = true;

    // Synchronize the MCU clock with the micro-ROS agent.  A millis()-based
    // stamp is interpreted as 1970 by ROS 2 and rejected as stale odometry.
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(
            &node,
            "teensy_base_controller",
            "",
            &support) != RCL_RET_OK) {
        return false;
    }
    node_initialized = true;

    if (rclc_publisher_init_default(
            &odom_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/odom") != RCL_RET_OK) {
        return false;
    }
    publisher_initialized = true;

    if (rclc_subscription_init_default(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) {
        return false;
    }
    subscription_initialized = true;

    if (rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator) != RCL_RET_OK) {
        return false;
    }
    executor_initialized = true;

    if (rclc_executor_add_subscription(
            &executor,
            &cmd_vel_sub,
            &cmd_vel_msg,
            &Telemetry::cmdVelCallback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    if (!nav_msgs__msg__Odometry__init(&odom_msg)) {
        return false;
    }
    odom_msg_initialized = true;

    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

    last_cmd_vel_time = millis();

    return true;
}

void Telemetry::destroyEntities()
{
    rcl_ret_t fini_ret = RCL_RET_OK;

    if (support_initialized) {
        rmw_context_t* rmw_context =
            rcl_context_get_rmw_context(&support.context);
        if (rmw_context != nullptr) {
            rmw_uros_set_context_entity_destroy_session_timeout(
                rmw_context, 0);
        }
    }

    if (executor_initialized) {
        rclc_executor_fini(&executor);
    }
    if (subscription_initialized && node_initialized) {
        fini_ret = rcl_subscription_fini(&cmd_vel_sub, &node);
    }
    if (publisher_initialized && node_initialized) {
        fini_ret = rcl_publisher_fini(&odom_pub, &node);
    }
    if (node_initialized) {
        fini_ret = rcl_node_fini(&node);
    }
    if (support_initialized) {
        rclc_support_fini(&support);
    }
    if (odom_msg_initialized) {
        nav_msgs__msg__Odometry__fini(&odom_msg);
    }

    support = rclc_support_t{};
    node = rcl_get_zero_initialized_node();
    odom_pub = rcl_get_zero_initialized_publisher();
    cmd_vel_sub = rcl_get_zero_initialized_subscription();
    executor = rclc_executor_get_zero_initialized_executor();
    odom_msg = nav_msgs__msg__Odometry{};

    support_initialized = false;
    node_initialized = false;
    publisher_initialized = false;
    subscription_initialized = false;
    executor_initialized = false;
    odom_msg_initialized = false;

    (void)fini_ret;
}

void Telemetry::stopRobot()
{
    cmd_v = 0.0f;
    cmd_w = 0.0f;
    calcWheelTarget();
}

bool Telemetry::isConnected() const
{
    return state == ConnectionState::AGENT_CONNECTED;
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

void Telemetry::publishOdom(float x, float y, float theta, float omega_r, float omega_l)
{
    if (!isConnected()) {
        return;
    }

    constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000LL;
    int64_t now_ns = rmw_uros_epoch_nanos();
    float vx = 0.5f * WHEEL_RADIUS * (omega_r + omega_l);
    float wz = WHEEL_RADIUS * (omega_r - omega_l) * WHEEL_BASE_INV;

    odom_msg.header.stamp.sec = now_ns / NANOSECONDS_PER_SECOND;
    odom_msg.header.stamp.nanosec = now_ns % NANOSECONDS_PER_SECOND;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0f;

    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(theta * 0.5f);

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.linear.z = 0.0f;
    odom_msg.twist.twist.angular.x = 0.0f;
    odom_msg.twist.twist.angular.y = 0.0f;
    odom_msg.twist.twist.angular.z = wz;

    rcl_ret_t publish_ret = rcl_publish(&odom_pub, &odom_msg, NULL);
    (void)publish_ret;
}
