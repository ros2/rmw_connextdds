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

#ifndef RMW_CONNEXTDDS__CONTEXT_HPP_
#define RMW_CONNEXTDDS__CONTEXT_HPP_

#include <stdio.h>

#include <limits>
#include <mutex>

#include "rmw_connextdds/dds_api.hpp"
#include "rmw_connextdds/log.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

#include "rcutils/strdup.h"


#if RMW_CONNEXT_HAVE_SCOPE_EXIT
#include "rcpputils/scope_exit.hpp"
#else
#include "scope_exit.hpp"
#endif /* RMW_CONNEXT_HAVE_SCOPE_EXIT */

struct rmw_context_impl_t
{
#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
    rmw_dds_common::Context common;
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */
    rmw_context_t *base;
    
    DDS_DomainParticipantFactory *factory;

    void *api;

    DDS_DomainId_t domain_id;
    DDS_DomainParticipant *participant;

    /* DDS publisher, subscriber used for ROS 2 publishers and subscriptions */
    DDS_Publisher *dds_pub;
    DDS_Subscriber *dds_sub;

    /* Built-in Discovery Readers */
    DDS_DataReader *dr_participants;
    DDS_DataReader *dr_publications;
    DDS_DataReader *dr_subscriptions;

    /* Keep track of whether the DomainParticipant is localhost only */
    bool localhost_only;

    /* Participant reference count*/
    size_t node_count{0};
    std::mutex initialization_mutex;

    /* Shutdown flag */
    bool is_shutdown{false};

    /* suffix for GUIDs to construct unique client/service ids
       (protected by initialization_mutex) */
    uint32_t client_service_id{0};

    rmw_context_impl_t(rmw_context_t *const base)
    :
#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
      common(),
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */
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
#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
        /* destructor relies on these being initialized properly */
        common.thread_is_running.store(false);
        common.graph_guard_condition = nullptr;
        common.pub = nullptr;
        common.sub = nullptr;
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */
    }

    // Initializes the participant, if it wasn't done already.
    // node_count is increased
    rmw_ret_t
    initialize_node(const bool localhost_only);

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

rmw_ret_t
rmw_connextdds_initialize_participant_factory_qos(
    rmw_context_impl_t *const ctx,
    DDS_DomainParticipantFactory *const factory);

#endif // RMW_CONNEXTDDS__CONTEXT_HPP_
