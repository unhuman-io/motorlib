# Copyright 2024 Figure AI, Inc
load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "obot-parser_test",
    size = "small",
    srcs = ["obot-parser_test.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//cpp_libs/gtest:gtest_main",
        "//firmware/lib/obot-parser",
    ],
)
