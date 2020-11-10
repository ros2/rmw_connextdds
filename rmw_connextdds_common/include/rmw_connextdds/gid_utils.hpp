// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_CONNEXTDDS__GID_UTILS_HPP_
#define RMW_CONNEXTDDS__GID_UTILS_HPP_

#include "rmw_connextdds/static_config.hpp"

#if !RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON

#include "rmw/types.h"

#include "rmw_connextdds/visibility_control.h"
#include "rmw_connextdds_common/msg/gid.hpp"
#include "rmw_connextdds_common/msg/node_entities_info.hpp"
#include "rmw_connextdds_common/msg/participant_entities_info.hpp"

namespace rmw_dds_common
{

namespace msg
{
  using Gid = rmw_connextdds_common::msg::Gid;
  using NodeEntitiesInfo = rmw_connextdds_common::msg::NodeEntitiesInfo;
  using ParticipantEntitiesInfo = rmw_connextdds_common::msg::ParticipantEntitiesInfo;
}

/// Comparator for rmw_gid_t, in order to use them as a key of a map
struct RMW_CONNEXTDDS_PUBLIC_TYPE Compare_rmw_gid_t
{
  /// Compare lhs with rhs.
  bool operator()(const rmw_gid_t & lhs, const rmw_gid_t & rhs) const;
};

/// Stream operator for rmw_gid_t
RMW_CONNEXTDDS_PUBLIC
std::ostream &
operator<<(std::ostream & ostream, const rmw_gid_t & gid);

/// operator== for rmw_gid_t
RMW_CONNEXTDDS_PUBLIC
bool
operator==(const rmw_gid_t & lhs, const rmw_gid_t & rhs);

/// Converts from rmw_gid_t to rmw_dds_common::msg::Gid
/**
 * For internal usage, both pointers are assumed to be valid.
 */
RMW_CONNEXTDDS_PUBLIC
void
convert_gid_to_msg(
  const rmw_gid_t * gid,
  rmw_dds_common::msg::Gid * msg_gid);

/// Converts from rmw_dds_common::msg::Gid to rmw_gid_t
/**
 * For internal usage, both pointers are supposed to be valid.
 */
RMW_CONNEXTDDS_PUBLIC
void
convert_msg_to_gid(
  const rmw_dds_common::msg::Gid * msg_gid,
  rmw_gid_t * gid);

}  // namespace rmw_dds_common

#endif /* !RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

#endif  // RMW_CONNEXTDDS__GID_UTILS_HPP___GID_UTILS_HPP_
