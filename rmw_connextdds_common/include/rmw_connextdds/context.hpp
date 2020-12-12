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

#ifndef RMW_CONNEXTDDS__CONTEXT_HPP_
#define RMW_CONNEXTDDS__CONTEXT_HPP_

#include <stdio.h>

#include <limits>
#include <mutex>
#include <string>

#include "rmw_connextdds/dds_api.hpp"
#include "rmw_connextdds/log.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/event.h"
#include "rmw/names_and_types.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_names_and_types.h"
#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/get_topic_endpoint_info.h"
#else
#include "rmw_connextdds/topic_endpoint_info_array.h"
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"
#else
#include "rmw_connextdds/context_common.hpp"
#include "rmw_connextdds_common/msg/participant_entities_info.hpp"
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

#include "rcutils/strdup.h"

#if RMW_CONNEXT_HAVE_SCOPE_EXIT
#include "rcpputils/scope_exit.hpp"
#else
#include "scope_exit.hpp"
#endif /* RMW_CONNEXT_HAVE_SCOPE_EXIT */

struct rmw_context_impl_t
{
  rmw_dds_common::Context common;
  rmw_context_t * base;

  DDS_DomainParticipantFactory * factory;

  void * api;

  DDS_DomainId_t domain_id;
  DDS_DomainParticipant * participant;

  /* DDS publisher, subscriber used for ROS 2 publishers and subscriptions */
  DDS_Publisher * dds_pub;
  DDS_Subscriber * dds_sub;

  /* Built-in Discovery Readers */
  DDS_DataReader * dr_participants;
  DDS_DataReader * dr_publications;
  DDS_DataReader * dr_subscriptions;

  /* Keep track of whether the DomainParticipant is localhost only */
  bool localhost_only;

  /* Global configuration for QoS profiles */
  std::string qos_library;
  std::string qos_ctx_name;
  std::string qos_ctx_namespace;

  /* Participant reference count*/
  size_t node_count{0};
  std::mutex initialization_mutex;

  /* Shutdown flag */
  bool is_shutdown{false};

  /* suffix for GUIDs to construct unique client/service ids
     (protected by initialization_mutex) */
  uint32_t client_service_id{0};

  explicit rmw_context_impl_t(rmw_context_t * const base)
  : common(),
    base(base),
    factory(nullptr),
    domain_id(RMW_CONNEXT_DEFAULT_DOMAIN),
    participant(nullptr),
    dds_pub(nullptr),
    dds_sub(nullptr),
    dr_participants(nullptr),
    dr_publications(nullptr),
    dr_subscriptions(nullptr),
#if RMW_CONNEXT_HAVE_LOCALHOST_ONLY
    localhost_only(base->options.localhost_only == RMW_LOCALHOST_ONLY_ENABLED)
#else
    localhost_only(false)
#endif /* RMW_CONNEXT_HAVE_LOCALHOST_ONLY */
  {
    /* destructor relies on these being initialized properly */
    common.thread_is_running.store(false);
    common.graph_guard_condition = nullptr;
    common.pub = nullptr;
    common.sub = nullptr;
  }

  // Initializes the participant, if it wasn't done already.
  // node_count is increased
  rmw_ret_t
  initialize_node(
    const char * const node_name,
    const char * const node_namespace,
    const bool localhost_only);

  // Destroys the participant, when node_count reaches 0.
  rmw_ret_t
  finalize_node();

  ~rmw_context_impl_t()
  {
    if (0u != this->node_count) {
      RMW_CONNEXT_LOG_ERROR("not all nodes finalized")
    }
  }

  uint32_t
  next_client_id();

  rmw_ret_t
  assert_topic(
    DDS_DomainParticipant * const participant,
    const char * const topic_name,
    const char * const type_name,
    const bool internal,
    DDS_Topic ** const topic,
    bool & created);

private:
  rmw_ret_t
  clean_up();
};

rmw_ret_t
rmw_connextdds_initialize_participant_factory_qos(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantFactory * const factory);

rmw_ret_t
rmw_connextdds_configure_security(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantQos * const qos);

#endif  // RMW_CONNEXTDDS__CONTEXT_HPP_
