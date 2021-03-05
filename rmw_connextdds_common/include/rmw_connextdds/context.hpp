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
#include <map>
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

#if RMW_CONNEXT_HAVE_QOS_PROFILE_API
#include "rmw/qos_profiles.h"
#include "rmw_dds_common/qos.hpp"
#endif /* RMW_CONNEXT_HAVE_QOS_PROFILE_API */

#include "rcutils/strdup.h"

#if RMW_CONNEXT_HAVE_SCOPE_EXIT
#include "rcpputils/scope_exit.hpp"
#else
#include "scope_exit.hpp"
#endif /* RMW_CONNEXT_HAVE_SCOPE_EXIT */

extern DDS_DomainParticipantFactory * RMW_Connext_gv_DomainParticipantFactory;
extern size_t RMW_Connext_gv_ContextCount;

enum class RMW_Connext_RequestReplyMapping
{
  Basic,
  Extended
};

struct rmw_context_impl_t
{
  rmw_dds_common::Context common;
  rmw_context_t * base;

  DDS_DomainParticipantFactory * factory;

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
  bool override_publish_mode;
  RMW_Connext_RequestReplyMapping request_reply_mapping;
  bool cyclone_compatible{false};
#if RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE
  bool old_rmw_compatible{false};
#endif /* RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE */
#if RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY
  bool fast_endp_discovery{true};
#endif /* RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY */
  /* Participant reference count*/
  size_t node_count{0};
  std::mutex initialization_mutex;

  /* Shutdown flag */
  bool is_shutdown{false};

  /* suffix for GUIDs to construct unique client/service ids
     (protected by initialization_mutex) */
  uint32_t client_service_id{0};

  std::map<std::string, RMW_Connext_MessageTypeSupport *> registered_types;
  std::mutex endpoint_mutex;

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

  ~rmw_context_impl_t()
  {
    if (0u != this->node_count) {
      RMW_CONNEXT_LOG_ERROR_A("not all nodes finalized: %lu", this->node_count)
    }
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

  // Initialize the DomainParticipant associated with the context.
  rmw_ret_t
  initialize_participant(const bool localhost_only);

  // Enable the DomainParticipant associated with the context.
  rmw_ret_t
  enable_participant();

  // Finalize the DomainParticipant associated with the context.
  rmw_ret_t
  finalize_participant();

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

  rmw_ret_t
  finalize();
};

rmw_ret_t
rmw_connextdds_initialize_participant_factory_qos(
  rmw_context_impl_t * const ctx);

rmw_ret_t
rmw_connextdds_configure_security(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantQos * const qos);

#endif  // RMW_CONNEXTDDS__CONTEXT_HPP_
