workspace(name = "osqp-eigen-bazel")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

# rules_cc
http_archive(
  name = "rules_cc",
  urls = ["https://github.com/bazelbuild/rules_cc/archive/262ebec3c2296296526740db4aefce68c80de7fa.zip"],
  strip_prefix = "rules_cc-262ebec3c2296296526740db4aefce68c80de7fa",
)

# Eigen
http_file(
    name = "eigen.BUILD",
    urls = ["https://raw.githubusercontent.com/tensorflow/tensorflow/v2.1.0/third_party/eigen.BUILD"],
    sha256 = "c8805ce048e79b788c8a9b5ed853c4a864dbd88d9c7b395e34adebba7293ad75",
)
http_archive(
    name = "eigen",
    build_file = "@eigen.BUILD//file:downloaded",
    url = "https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip",
    sha256 = "e09b89aae054e9778ee3f606192ee76d645eec82c402c01c648b1fe46b6b9857",
    strip_prefix = "eigen-3.3.7",
)

# osqp
local_repository(
    name = "osqp_lib",
    path = "ThirdParty/osqp/",
)

# osqp_eigen
local_repository(
    name = "osqp_eigen_lib",
    path = "ThirdParty/osqp_eigen/",
)

