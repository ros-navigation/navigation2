// Copyright (c) 2023 Samsung R&D Institute Russia
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

#ifndef NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_
#define NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_

#include <uuid/uuid.h>
#include <string>

namespace nav2_map_server
{

// ---------- Working with UUID-s ----------

/**
 * @brief Converts UUID from input array to unparsed string
 * @param uuid Input UUID in array format
 * @return Unparsed UUID string
 */
inline std::string unparseUUID(const unsigned char * uuid)
{
  char uuid_str[37];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_
