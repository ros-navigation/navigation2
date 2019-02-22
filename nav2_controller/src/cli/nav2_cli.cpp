// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "nav2_controller/nav2_controller_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  nav2_controller::Nav2ControllerClient client;

#if 0
  if (!strcmp(argv[1], "startup")) {
    client.startup();
  } else if (!strcmp(argv[1], "shutdown")) {
    client.shutdown();
  } else if (!strcmp(argv[1], "pause")) {
    client.pause();
  } else if (!strcmp(argv[1], "resume")) {
    client.resume();
  }
#else
  std::this_thread::sleep_for(10s);
  client.startup();
  std::this_thread::sleep_for(5s);
  client.shutdown();
#endif

  rclcpp::shutdown();
  return 0;
}

