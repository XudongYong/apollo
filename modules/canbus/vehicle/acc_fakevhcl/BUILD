load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "acc_fakevhcl_vehicle_factory",
    srcs = [
        "acc_fakevhcl_vehicle_factory.cc",
    ],
    hdrs = [
        "acc_fakevhcl_vehicle_factory.h",
    ],
    deps = [
        ":acc_fakevhcl_controller",
        ":acc_fakevhcl_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "acc_fakevhcl_message_manager",
    srcs = [
        "acc_fakevhcl_message_manager.cc",
    ],
    hdrs = [
        "acc_fakevhcl_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/acc_fakevhcl/protocol:canbus_acc_fakevhcl_protocol",
    ],
)

cc_library(
    name = "acc_fakevhcl_controller",
    srcs = [
        "acc_fakevhcl_controller.cc",
    ],
    hdrs = [
        "acc_fakevhcl_controller.h",
    ],
    deps = [
        ":acc_fakevhcl_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/acc_fakevhcl/protocol:canbus_acc_fakevhcl_protocol",
    ],
)

cpplint()
