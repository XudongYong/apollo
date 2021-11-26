load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ACC_Component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":acc_component_lib"],
)

cc_library(
    name = "acc_component_lib",
    srcs = ["acc_component.cc"],
    hdrs = ["acc_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:conti_radar_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
         "//modules/common/adapters:adapter_gflags",
        ":acc_lib",
    ],    
)

cc_library(
    name = "acc_lib",
    srcs = [
        "ACC.c",
        "DVARead.c",
        "DVAWrite.c",
        "GuiCmd.c",
        "ApoClnt.c",
    ],
    hdrs = [
        "ACC.h",
        "apo.h",
        "ApoClnt.h",
        "DVA.h",
        "GuiCmd.h",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":libapo"
    ],
    linkstatic = 1,
)

cc_library(
   name = "libapo",
   srcs = ["libapo/libapo-client-linux64.a"],
   hdrs = ["apo.h"] 
)

cpplint()
