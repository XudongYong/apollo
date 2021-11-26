load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "poc_vehicle_vehicle_factory",
    srcs = [
        "poc_vehicle_vehicle_factory.cc",
    ],
    hdrs = [
        "poc_vehicle_vehicle_factory.h",
    ],
    deps = [
        ":poc_vehicle_controller",
        ":poc_vehicle_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "poc_vehicle_message_manager",
    srcs = [
        "poc_vehicle_message_manager.cc",
    ],
    hdrs = [
        "poc_vehicle_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/poc_vehicle/protocol:canbus_poc_vehicle_protocol",
    ],
)

cc_library(
    name = "poc_vehicle_controller",
    srcs = [
        "poc_vehicle_controller.cc",
    ],
    hdrs = [
        "poc_vehicle_controller.h",
    ],
    deps = [
        ":poc_vehicle_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/poc_vehicle/protocol:canbus_poc_vehicle_protocol",
    ],
)

cpplint()
