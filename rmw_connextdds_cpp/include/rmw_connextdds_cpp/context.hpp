/******************************************************************************
 *
 * (c) 2020 Copyright, Real-Time Innovations, Inc. (RTI)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef RMW_CONNEXT__CONTEXT_HPP_
#define RMW_CONNEXT__CONTEXT_HPP_

#include "rmw_connextdds_cpp/static_config.h"
#include "rmw_connextdds_cpp/log.hpp"

#include <limits>

#include <stdio.h>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"

#include "rcutils/strdup.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"


#if RMW_CONNEXT_RELEASE == RMW_CONNEXT_RELEASE_FOXY
#include "scope_exit.hpp"
#else
#include "rcpputils/scope_exit.hpp"
#endif /* RMW_CONNEXT_RELEASE == RMW_CONNEXT_RELEASE_FOXY */

#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO
#include "rti_me_c.h"
#include "disc_dpde/disc_dpde_discovery_plugin.h"
#include "wh_sm/wh_sm_history.h"
#include "rh_sm/rh_sm_history.h"
#include "netio/netio_udp.h"
#include "netio_shmem/netio_shmem.h"
// #include "sec_core/sec_core_c.h"
#include "rmw_connextdds_cpp/rtime_ext.h"
#include "REDASequence.h"
#elif RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO
#include "ndds/ndds_c.h"
#else
#error "invalid DDS API selected"
#endif /* RMW_CONNEXT_DDS_API */

#ifndef UNUSED_ARG
#define UNUSED_ARG(arg_)        (void)(arg_)
#endif /* UNUSED_ARG */

extern const char * const RMW_CONNEXTDDS_ID;
extern const char * const RMW_CONNEXTDDS_SERIALIZATION_FORMAT;

struct rmw_context_impl_t
{
    rmw_dds_common::Context common;
    rmw_context_t *base;
    
    DDS_DomainParticipantFactory *factory;

#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO
    bool rt_whsm;
    bool rt_rhsm;
    bool rt_udp;
    bool rt_shmem;
    bool rt_dpde;
    bool rt_sec;
    struct UDP_InterfaceFactoryProperty *udp_property;
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO */

#if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_ROLLING
    DDS_DomainId_t domain_id;
#endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_ROLLING */
    DDS_DomainParticipant *participant;

    /* DDS publisher, subscriber used for ROS 2 publishers and subscriptions */
    DDS_Publisher *dds_pub;
    DDS_Subscriber *dds_sub;

    /* Built-in Discovery Readers */
    DDS_DataReader *dr_participants;
    DDS_DataReader *dr_publications;
    DDS_DataReader *dr_subscriptions;

    /* Participant reference count*/
    size_t node_count{0};
    std::mutex initialization_mutex;

    /* Shutdown flag */
    bool is_shutdown{false};

    /* suffix for GUIDs to construct unique client/service ids
       (protected by initialization_mutex) */
    uint32_t client_service_id{0};

    rmw_context_impl_t(rmw_context_t *const base)
    : common(),
      base(base),
      factory(nullptr),
#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO
      rt_whsm(false),
      rt_rhsm(false),
      rt_udp(false),
      rt_shmem(false),
      rt_dpde(false),
      rt_sec(false),
      udp_property(nullptr),
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO */
#if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_ROLLING
      domain_id(0),
#endif /* RMW_CONNEXT_RELEASE == RMW_CONNEXT_RELEASE_ROLLING */
      participant(nullptr),
      dds_pub(nullptr),
      dds_sub(nullptr),
      dr_participants(nullptr),
      dr_publications(nullptr),
      dr_subscriptions(nullptr)
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
    initialize_node();

    // Destroys the participant, when node_count reaches 0.
    rmw_ret_t
    finalize_node();

    ~rmw_context_impl_t()
    {
        if (0u != this->node_count)
        {
            RMW_CONNEXT_LOG_ERROR("not all nodes finalized")
        }
    }

    uint32_t
    next_client_id();

    rmw_ret_t
    assert_topic(
        DDS_DomainParticipant *const participant,
        const char *const topic_name,
        const char *const type_name,
        const bool internal,
        DDS_Topic **const topic,
        bool &created);

private:
    rmw_ret_t
    clean_up();

};

#endif /* RMW_CONNEXT__CONTEXT_HPP_ */
