// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef CONTROL__CONTROLTASKCLIENT_HPP_
#define CONTROL__CONTROLTASKCLIENT_HPP_

#include "task/TaskClient.hpp"

typedef TaskClient<std_msgs::msg::String, std_msgs::msg::String> ControlTaskClient;

#endif  // CONTROL__CONTROLTASKCLIENT_HPP_
