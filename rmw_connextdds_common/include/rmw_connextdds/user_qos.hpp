// Copyright 2020 Real-Time Innovations, Inc. (RTI)
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

#ifndef RMW_CONNEXTDDS__USER_QOS_HPP_
#define RMW_CONNEXTDDS__USER_QOS_HPP_

#include <memory>

#include "rmw_connextdds/dds_api.hpp"

namespace rmw_connextdds
{

class UserQos
{
public:
  RMW_CONNEXTDDS_PUBLIC
  virtual
  ~UserQos() = default;

  RMW_CONNEXTDDS_PUBLIC
  virtual
  rmw_ret_t configure_context(
    const rmw_context_t * const context,
    DDS_DomainParticipantQos & qos) = 0;

  RMW_CONNEXTDDS_PUBLIC
  virtual
  rmw_ret_t configure_subscription(
    const rmw_context_t * const context,
    const rmw_node_t * const node,
    DDS_TopicDescription * const topic_desc,
    DDS_DataReaderQos & qos) = 0;

  RMW_CONNEXTDDS_PUBLIC
  virtual
  rmw_ret_t configure_publisher(
    const rmw_context_t * const context,
    const rmw_node_t * const node,
    DDS_Topic * const topic,
    DDS_DataWriterQos & qos) = 0;
};

typedef std::shared_ptr<UserQos> (*UserQosLoadFn)(void);

}  // namespace rmw_connextdds

#endif  // RMW_CONNEXTDDS__USER_QOS_HPP_
