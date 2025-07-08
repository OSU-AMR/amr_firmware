#include "ros.h"

#include "controller.h"
#include "ir.h"

#include "driver/cd74hc4051.h"
#include "driver/thermistor.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <amr_msgs/msg/encoder.h>
#include <amr_msgs/msg/firmware_status.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/u_int16.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define LEFT_IR_PUBLISHER_NAME "state/ir/left"
#define RIGHT_IR_PUBLISHER_NAME "state/ir/right"
#define BACK_IR_PUBLISHER_NAME "state/ir/back"
#define ENCODER_PUBLISHER_NAME "state/encoder"

#define MOT0_THERM_PUBLISHER_NAME "state/motor_left/temperature"
#define MOT1_THERM_PUBLISHER_NAME "state/motor_right/temperature"
#define ESC0_THERM_PUBLISHER_NAME "state/esc_left/temperature"
#define ESC1_THERM_PUBLISHER_NAME "state/esc_right/temperature"

#define LEFT_BRAKING_SUBSCRIBER_NAME "command/left_braking_voltage"
#define RIGHT_BRAKING_SUBSCRIBER_NAME "command/right_braking_voltage"
#define VELOCITY_SLEW_SUBSCRIBER_NAME "command/velocity_slew"

bool ros_connected = false;

// Core Variables
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t heartbeat_publisher;
int failed_heartbeats = 0;

// Node specific Variables
rcl_publisher_t firmware_status_publisher;
rcl_subscription_t killswtich_subscriber;
std_msgs__msg__Bool killswitch_msg;
// TODO: Add node specific items here
rcl_publisher_t left_ir_publisher;
rcl_publisher_t right_ir_publisher;
rcl_publisher_t back_ir_publisher;

rcl_publisher_t encoder_publisher;

rcl_publisher_t mot0_therm_publisher;
rcl_publisher_t mot1_therm_publisher;
rcl_publisher_t esc0_therm_publisher;
rcl_publisher_t esc1_therm_publisher;

rcl_subscription_t left_braking_subscriber;
std_msgs__msg__Float32 left_braking_msg;
rcl_subscription_t right_braking_subscriber;
std_msgs__msg__Float32 right_braking_msg;
rcl_subscription_t velocity_slew_subscriber;
std_msgs__msg__Float32 velocity_slew_msg;

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void left_braking_subscription_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *) msgin;
    controller_set_braking_voltage(0, msg->data);
}

static void right_braking_subscription_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *) msgin;
    controller_set_braking_voltage(1, msg->data);
}

static void velocity_slew_subscription_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *) msgin;
    controller_set_slew_rps2(msg->data);
}

// TODO: Add in node specific tasks here

// ========================================
// Public Task Methods (called in main tick)
// ========================================

rcl_ret_t ros_update_firmware_status(uint8_t client_id) {
    amr_msgs__msg__FirmwareStatus status_msg;
    status_msg.board_name.data = PICO_BOARD;
    status_msg.board_name.size = strlen(PICO_BOARD);
    status_msg.board_name.capacity = status_msg.board_name.size + 1;  // includes NULL byte

// Select bus ID depending on the transport used
#ifdef MICRO_ROS_TRANSPORT_CAN
    status_msg.bus_id = __CONCAT(CAN_BUS_NAME, _ID);
#elif MICRO_ROS_TRANSPORT_ETH
    status_msg.bus_id = ETHERNET_BUS_ID;
#else
    status_msg.bus_id = 0;
#endif

    status_msg.client_id = client_id;
    status_msg.uptime_ms = to_ms_since_boot(get_absolute_time());
    status_msg.version_major = MAJOR_VERSION;
    status_msg.version_minor = MINOR_VERSION;
    status_msg.version_release_type = RELEASE_TYPE;
    status_msg.faults = *fault_list_reg;
    status_msg.kill_switches_enabled = 0;
    status_msg.kill_switches_asserting_kill = 0;
    status_msg.kill_switches_needs_update = 0;
    status_msg.kill_switches_timed_out = 0;

    for (int i = 0; i < NUM_KILL_SWITCHES; i++) {
        if (kill_switch_states[i].enabled) {
            status_msg.kill_switches_enabled |= (1 << i);
        }

        if (kill_switch_states[i].asserting_kill) {
            status_msg.kill_switches_asserting_kill |= (1 << i);
        }

        if (kill_switch_states[i].needs_update) {
            status_msg.kill_switches_needs_update |= (1 << i);
        }

        if (kill_switch_states[i].needs_update && time_reached(kill_switch_states[i].update_timeout)) {
            status_msg.kill_switches_timed_out |= (1 << i);
        }
    }

    RCSOFTRETCHECK(rcl_publish(&firmware_status_publisher, &status_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_heartbeat_pulse(uint8_t client_id) {
    std_msgs__msg__Int8 heartbeat_msg;
    heartbeat_msg.data = client_id;
    rcl_ret_t ret = rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL);
    if (ret != RCL_RET_OK) {
        failed_heartbeats++;

        if (failed_heartbeats > MAX_MISSSED_HEARTBEATS) {
            ros_connected = false;
        }
    }
    else {
        failed_heartbeats = 0;
    }

    RCSOFTRETCHECK(ret);

    return RCL_RET_OK;
}

rcl_ret_t ros_publish_ir_sensors() {
    std_msgs__msg__UInt16 left_ir_msg, right_ir_msg, back_ir_msg;

    left_ir_msg.data = ir_read(IR2_PIN);
    right_ir_msg.data = ir_read(IR1_PIN);
    back_ir_msg.data = ir_read(IR0_PIN);

    RCSOFTRETCHECK(rcl_publish(&left_ir_publisher, &left_ir_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&right_ir_publisher, &right_ir_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&back_ir_publisher, &back_ir_msg, NULL));

    return RCL_RET_OK;
}

static inline void nanos_to_timespec(int64_t time_nanos, struct timespec *ts) {
    ts->tv_sec = time_nanos / 1000000000;
    ts->tv_nsec = time_nanos % 1000000000;
}

rcl_ret_t ros_publish_encoders() {
    amr_msgs__msg__Encoder encoder_msg;
    struct timespec ts;
    nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);
    encoder_msg.stamp.sec = ts.tv_sec;
    encoder_msg.stamp.nanosec = ts.tv_nsec;

    const float *encoders_angle = controller_get_encoders_angle();
    encoder_msg.left_angle = encoders_angle[0];
    encoder_msg.right_angle = encoders_angle[1];

    const float *encoders_vel = controller_get_encoders_vel();
    encoder_msg.left_velocity = encoders_vel[0];
    encoder_msg.right_velocity = encoders_vel[1];

    RCSOFTRETCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_publish_thermistors() {
    std_msgs__msg__Float32 mot0_msg, mot1_msg, esc0_msg, esc1_msg;

    mot0_msg.data = thermistor_get_c(multiplexer_decode_analog(MOT0_THERM_MUX_NUM), THERMISTOR_PROFILE_CHASSIS);
    mot1_msg.data = thermistor_get_c(multiplexer_decode_analog(MOT1_THERM_MUX_NUM), THERMISTOR_PROFILE_CHASSIS);
    esc0_msg.data = thermistor_get_c(multiplexer_decode_analog(ESC0_THERM_MUX_NUM), THERMISTOR_PROFILE_BOARD);
    esc1_msg.data = thermistor_get_c(multiplexer_decode_analog(ESC1_THERM_MUX_NUM), THERMISTOR_PROFILE_BOARD);

    RCSOFTRETCHECK(rcl_publish(&mot0_therm_publisher, &mot0_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&mot1_therm_publisher, &mot1_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&esc0_therm_publisher, &esc0_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&esc1_therm_publisher, &esc1_msg, NULL));

    return RCL_RET_OK;
}

// TODO: Add in node specific tasks here

// ========================================
// ROS Core
// ========================================

rcl_ret_t ros_init() {
    // ROS Core Initialization
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCRETCHECK(rclc_node_init_default(&node, PICO_TARGET_NAME, ROBOT_NAMESPACE, &support));

    // Node Initialization
    RCRETCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&firmware_status_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(amr_msgs, msg, FirmwareStatus),
                                           FIRMWARE_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&left_ir_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), LEFT_IR_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &right_ir_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), RIGHT_IR_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&back_ir_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), BACK_IR_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &mot0_therm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), MOT0_THERM_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &mot1_therm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), MOT1_THERM_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &esc0_therm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ESC0_THERM_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &esc1_therm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ESC1_THERM_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(amr_msgs, msg, Encoder), ENCODER_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), KILLSWITCH_SUBCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&left_braking_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                              LEFT_BRAKING_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&right_braking_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                              RIGHT_BRAKING_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&velocity_slew_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                              VELOCITY_SLEW_SUBSCRIBER_NAME));

    // Executor Initialization
    const int executor_num_handles = 4;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));

    RCRETCHECK(rclc_executor_add_subscription(&executor, &left_braking_subscriber, &left_braking_msg,
                                              &left_braking_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &right_braking_subscriber, &right_braking_msg,
                                              &right_braking_subscription_callback, ON_NEW_DATA));

    RCRETCHECK(rclc_executor_add_subscription(&executor, &velocity_slew_subscriber, &velocity_slew_msg,
                                              &velocity_slew_subscription_callback, ON_NEW_DATA));

    // TODO: Modify this method with node specific objects

    // Note: Code in executor callbacks should be kept to a minimum
    // It should set whatever flags are necessary and get out
    // And it should *NOT* try to perform any communiations over ROS, as this can lead to watchdog timeouts
    // in the event that specific request times out

    return RCL_RET_OK;
}

void ros_spin_executor(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    // TODO: Modify to clean up anything you have opened in init here to avoid memory leaks

    RCSOFTCHECK(rcl_subscription_fini(&killswtich_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&left_braking_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&right_braking_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&velocity_slew_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&left_ir_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&right_ir_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&back_ir_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&encoder_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&mot0_therm_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&mot1_therm_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&esc0_therm_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&esc1_therm_publisher, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    ros_connected = false;
}

bool is_ros_connected(void) {
    return ros_connected;
}

bool ros_ping(void) {
    ros_connected = rmw_uros_ping_agent(RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT, 1) == RCL_RET_OK;
    return ros_connected;
}
