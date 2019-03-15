// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__GLOBAL_LOCALIZATION_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__GLOBAL_LOCALIZATION_SERVICE_CLIENT_HPP_

#include <string>

#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_util
{

class GlobalLocalizationServiceClient : public nav2_util::ServiceClient<std_srvs::srv::Empty>
{
public:
  explicit GlobalLocalizationServiceClient(const std::string & parent_node_name)
  : ServiceClient<std_srvs::srv::Empty>("global_localization", parent_node_name)
  {
  }

  using globalLocalizationServiceRequest =
    nav2_util::ServiceClient<std_srvs::srv::Empty>::RequestType;
  using globalLocalizationServiceResponse =
    nav2_util::ServiceClient<std_srvs::srv::Empty>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__GLOBAL_LOCALIZATION_SERVICE_CLIENT_HPP_
