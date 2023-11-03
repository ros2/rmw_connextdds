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

#include <map>
#include <mutex>
#include <regex>
#include <string>

#include "rmw_connextdds/dds_api.hpp"
#include "rmw_connextdds/log.hpp"

#include "rmw_dds_common/context.hpp"

#include "rmw/discovery_options.h"
#include "rmw/init.h"
#include "rmw/ret_types.h"

#include "rmw/impl/cpp/macros.hpp"

#include "rmw/rmw.h"

extern DDS_DomainParticipantFactory * RMW_Connext_gv_DomainParticipantFactory;
extern size_t RMW_Connext_gv_ContextCount;

enum class RMW_Connext_RequestReplyMapping
{
  Basic,
  Extended
};

// Definition of struct rmw_context_impl_s as declared in rmw/init.h
struct rmw_context_impl_s
{
public:
  rmw_dds_common::Context common;
  std::mutex common_mutex;
  rmw_context_t * base;

  DDS_DomainId_t domain_id;
  DDS_DomainParticipant * participant;

  /* DDS publisher, subscriber used for ROS 2 publishers and subscriptions */
  DDS_Publisher * dds_pub;
  DDS_Subscriber * dds_sub;

  /* Built-in Discovery Readers */
  DDS_DataReader * dr_participants;
  DDS_DataReader * dr_publications;
  DDS_DataReader * dr_subscriptions;

  /* Global configuration for QoS profiles */
  bool use_default_publish_mode;
  RMW_Connext_RequestReplyMapping request_reply_mapping;
  bool cyclone_compatible{false};
#if RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE
  bool legacy_rmw_compatible{false};
#endif /* RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE */
#if RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY
  bool fast_endp_discovery{true};
#endif /* RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY */
#if RMW_CONNEXT_DEFAULT_LARGE_DATA_OPTIMIZATIONS
  bool optimize_large_data{true};
#endif /* RMW_CONNEXT_DEFAULT_LARGE_DATA_OPTIMIZATIONS */
#if RMW_CONNEXT_DEFAULT_RELIABILITY_OPTIMIZATIONS
  bool optimize_reliability{true};
#endif /* RMW_CONNEXT_DEFAULT_RELIABILITY_OPTIMIZATIONS */

  enum class participant_qos_override_policy_t
  {
    // Always override the default DomainParticipantQoS obtained at runtime from
    // Connext with RMW-specific configuration. This will include settings derived
    // from ROS 2 configuration parameters (e.g. "localhost_only", or "enclave"),
    // but also some additional configurations that the RMW performs arbitrarly
    // to improve the out of the box experience. Note that some of these customizations
    // can also be disabled individually (e.g. fast endpoint discovery).
    All,
    // Only perform basic modifications on the default DomainParticipantQos value
    // based on ROS 2 configuration parameters (e.g. "localhost only", and "enclave").
    // All other RMW-specific customizations will not be applied.
    Basic,
    // Use the default DomainParticipantQoS returned by Connext without any modification.
    Never,
  };

  participant_qos_override_policy_t participant_qos_override_policy;

  enum class endpoint_qos_override_policy_t
  {
    // Use default QoS policy got from the DDS qos profile file applying topic filters
    // and apply the ROS settings on top of it.
    Always,
    // Use default QoS policy got from the DDS qos profile file applying topic filters
    // without any changes for the topics that matches the provided regex expressions.
    // Apply the ROS settings on top of it for the ones that doesn't match.
    DDSTopics,
    // Use default QoS policy got from the DDS qos profile file applying topic filters
    // without any changes.
    Never,
  };

  endpoint_qos_override_policy_t endpoint_qos_override_policy;
  std::regex endpoint_qos_override_policy_topics_regex;

  struct DDS_StringSeq initial_peers = DDS_SEQUENCE_INITIALIZER;

  /* Participant reference count*/
  size_t node_count{0};
  std::mutex initialization_mutex;

  /* Shutdown flag */
  bool is_shutdown{false};

  std::map<std::string, RMW_Connext_MessageTypeSupport *> registered_types;
  std::mutex endpoint_mutex;

  explicit rmw_context_impl_s(rmw_context_t * const base);

  ~rmw_context_impl_s();

  // Initializes the participant, if it wasn't done already.
  // node_count is increased
  rmw_ret_t
  initialize_node();

  // Destroys the participant, when node_count reaches 0.
  rmw_ret_t
  finalize_node();

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

private:
  rmw_ret_t
  configure_security(DDS_DomainParticipantQos * const qos);

  rmw_ret_t
  initialize_discovery_options(DDS_DomainParticipantQos & dp_qos);

  rmw_ret_t
  initialize_participant_qos(DDS_DomainParticipantQos & dp_qos);

  // Initialize the DomainParticipant associated with the context.
  rmw_ret_t
  initialize_participant();

  // Enable the DomainParticipant associated with the context.
  rmw_ret_t
  enable_participant();

  uint32_t
  next_client_id();

  // Finalize the DomainParticipant associated with the context.
  rmw_ret_t
  finalize_participant();

  /* Manage the memory of the domain tag */
  char * domain_tag{nullptr};

  /* suffix for GUIDs to construct unique client/service ids
     (protected by initialization_mutex) */
  uint32_t client_service_id{0};
};

#endif  // RMW_CONNEXTDDS__CONTEXT_HPP_
