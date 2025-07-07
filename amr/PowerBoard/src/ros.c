#include "ros.h"

#include "driver/cd74hc4051.h"
#include "driver/mfrc522.h"
#include "driver/thermistor.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <amr_msgs/msg/firmware_status.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define RFID_PUBLISHER_NAME "state/rfid"
#define BATTERY_VOLTAGE_PUBLISHER_NAME "state/battery/voltage"
#define BATTERY_TEMP_PUBLISHER_NAME "state/battery/temperature"

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

rcl_publisher_t rfid_publisher;
uint8_t last_tag[MAX_UID_SIZE];

rcl_publisher_t battery_voltage_publisher;
rcl_publisher_t battery_temp_publisher;

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
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

// TODO: Add in node specific tasks here

bool arrs_equal(uint8_t arr1[], uint8_t arr2[], uint8_t len) {
    for (int i = 0; i < len; i++) {
        if (arr1[i] != arr2[i])
            return false;
    }

    return true;
}

rcl_ret_t ros_publish_rfid(uint8_t bytes[], uint8_t size) {
    if (arrs_equal(bytes, last_tag, MAX_UID_SIZE))
        return RCL_RET_OK;
    memcpy(last_tag, bytes, MAX_UID_SIZE);  // We have a new tag, so copy it in for later

    std_msgs__msg__String rfid_msg;
    const uint16_t str_size = 2 * size;

    char tag_code[str_size];
    for (uint8_t i = 0; i < size; i++) {
        sprintf(&tag_code[2 * i], "%02hhx", bytes[i]);
    }

    rfid_msg.data.data = tag_code;
    rfid_msg.data.size = str_size;
    rfid_msg.data.capacity = str_size + 1;

    RCSOFTRETCHECK(rcl_publish(&rfid_publisher, &rfid_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_publish_battery_state(float voltage) {
    std_msgs__msg__Float32 voltage_msg, temp_msg;
    voltage_msg.data = voltage;
    temp_msg.data = thermistor_get_c(multiplexer_decode_analog(BATT_TEMP_MUX_NUM), THERMISTOR_PROFILE_CHASSIS);

    RCSOFTRETCHECK(rcl_publish(&battery_voltage_publisher, &voltage_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&battery_temp_publisher, &temp_msg, NULL));

    return RCL_RET_OK;
}

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

    RCRETCHECK(rclc_publisher_init_default(&rfid_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                           RFID_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&battery_voltage_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                           BATTERY_VOLTAGE_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&battery_temp_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                           BATTERY_TEMP_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), KILLSWITCH_SUBCRIBER_NAME));

    // Executor Initialization
    const int executor_num_handles = 1;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));

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
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&rfid_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&battery_voltage_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&battery_temp_publisher, &node));
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
