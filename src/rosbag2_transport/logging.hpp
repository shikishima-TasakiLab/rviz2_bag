// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modifications copyright (C) 2024 <shikishima-TasakiLab>

#ifndef RVIZ2_BAG__ROSBAG2_TRANSPORT__LOGGING_HPP_
#define RVIZ2_BAG__ROSBAG2_TRANSPORT__LOGGING_HPP_

#include <sstream>
#include <string>

#include <rcutils/logging_macros.h>

#define ROSBAG2_TRANSPORT_PACKAGE_NAME "ROSBAG2_TRANSPORT"

#define ROSBAG2_TRANSPORT_LOG_INFO(...) \
  RCUTILS_LOG_INFO_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, __VA_ARGS__)

#define ROSBAG2_TRANSPORT_LOG_INFO_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    RCUTILS_LOG_INFO_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, "%s", __ss.str().c_str()); \
} while (0)

#define ROSBAG2_TRANSPORT_LOG_ERROR(...) \
  RCUTILS_LOG_ERROR_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, __VA_ARGS__)

#define ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    RCUTILS_LOG_ERROR_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, "%s", __ss.str().c_str()); \
} while (0)

#define ROSBAG2_TRANSPORT_LOG_WARN(...) \
  RCUTILS_LOG_WARN_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, __VA_ARGS__)

#define ROSBAG2_TRANSPORT_LOG_WARN_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    RCUTILS_LOG_WARN_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, "%s", __ss.str().c_str()); \
} while (0)

#define ROSBAG2_TRANSPORT_LOG_DEBUG(...) \
  RCUTILS_LOG_DEBUG_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, __VA_ARGS__)

#define ROSBAG2_TRANSPORT_LOG_DEBUG_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    RCUTILS_LOG_DEBUG_NAMED(ROSBAG2_TRANSPORT_PACKAGE_NAME, "%s", __ss.str().c_str()); \
} while (0)

#endif  // RVIZ2_BAG__ROSBAG2_TRANSPORT__LOGGING_HPP_
