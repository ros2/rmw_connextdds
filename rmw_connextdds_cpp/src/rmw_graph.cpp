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

#include "rmw_connextdds_cpp/discovery.hpp"
#include "rmw_connextdds_cpp/graph_cache.hpp"

rmw_ret_t
rmw_connextdds_graph_initialize(rmw_context_impl_t *const ctx)
{
    rmw_qos_profile_t pubsub_qos = rmw_qos_profile_default;
    pubsub_qos.avoid_ros_namespace_conventions = true;
    pubsub_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    pubsub_qos.depth = 5;
    pubsub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    pubsub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

    /* Create RMW publisher/subscription/guard condition used by rmw_dds_common
        discovery */
    rmw_publisher_options_t publisher_options =
        rmw_get_default_publisher_options();
    
    const rosidl_message_type_support_t *const type_supports_partinfo =
        rosidl_typesupport_cpp::get_message_type_support_handle<
            rmw_dds_common::msg::ParticipantEntitiesInfo>();
    
    const char *const topic_name_partinfo = "ros_discovery_info";

    RMW_CONNEXT_LOG_DEBUG("creating discovery writer")

    ctx->common.pub =
        rmw_connextdds_create_publisher(
            ctx,
            nullptr /* node */,
            ctx->participant,
            ctx->dds_pub,
            type_supports_partinfo,
            topic_name_partinfo,
            &pubsub_qos,
            &publisher_options,
            true /* internal */);
    
    if (nullptr == ctx->common.pub)
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to create publisher for ParticipantEntityInfo")
        return RMW_RET_ERROR;
    }

    rmw_subscription_options_t subscription_options =
        rmw_get_default_subscription_options();
    subscription_options.ignore_local_publications = true;
    
    pubsub_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    RMW_CONNEXT_LOG_DEBUG("creating discovery subscriber")

    ctx->common.sub =
        rmw_connextdds_create_subscriber(
            ctx,
            nullptr /* node */,
            ctx->participant,
            ctx->dds_sub,
            type_supports_partinfo,
            topic_name_partinfo,
            &pubsub_qos,
            &subscription_options,
            true /* internal */);
    if (nullptr == ctx->common.sub)
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to create subscriber for ParticipantEntityInfo")
        return RMW_RET_ERROR;
    }

    RMW_CONNEXT_LOG_DEBUG("configuring discovery")

    ctx->common.graph_guard_condition =
        rmw_connextdds_create_guard_condition();
    if (nullptr == ctx->common.graph_guard_condition)
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to create graph guard condition")
        return RMW_RET_BAD_ALLOC;
    }

    ctx->common.graph_cache.set_on_change_callback(
        [gcond = ctx->common.graph_guard_condition]()
        {
            rmw_ret_t ret = rmw_trigger_guard_condition(gcond);
            if (ret != RMW_RET_OK)
            {
                RMW_CONNEXT_LOG_ERROR(
                    "failed to trigger graph cache on_change_callback")
            }
        });

    rmw_connextdds_get_entity_gid(ctx->participant, ctx->common.gid);

    ctx->common.graph_cache.add_participant(
        ctx->common.gid, ctx->base->options.enclave);
    
    ctx->dr_participants =
        rmw_connextdds_get_builtin_reader_participants(ctx);
    
    ctx->dr_publications =
        rmw_connextdds_get_builtin_reader_publications(ctx);
    
    ctx->dr_subscriptions =
        rmw_connextdds_get_builtin_reader_subscriptions(ctx);

#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO
    if (nullptr == ctx->dr_participants ||
        nullptr == ctx->dr_publications ||
        nullptr == ctx->dr_subscriptions)
    {
        RMW_CONNEXT_LOG_ERROR("failed to lookup built-in data-readers")
        return RMW_RET_ERROR;
    }
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_enable(rmw_context_impl_t *const ctx)
{
    auto pub = (RMW_Connext_Publisher*)ctx->common.pub->data;
    if (RMW_RET_OK != pub->enable())
    {
        return RMW_RET_ERROR;
    }

    auto sub = (RMW_Connext_Subscriber*)ctx->common.sub->data;
    if (RMW_RET_OK != sub->enable())
    {
        return RMW_RET_ERROR;
    }

    if (RMW_RET_OK != rmw_connextdds_enable_builtin_readers(ctx))
    {
        return RMW_RET_ERROR;
    }

    rmw_ret_t ret = rmw_connextdds_discovery_thread_start(ctx);
    if (RMW_RET_OK != ret)
    {
        RMW_CONNEXT_LOG_ERROR("failed to start discovery thread")
        return ret;
    }

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_finalize(rmw_context_impl_t *const ctx)
{
    if (RMW_RET_OK != rmw_connextdds_discovery_thread_stop(ctx))
    {
        RMW_CONNEXT_LOG_ERROR("failed to stop discovery thread")
        return RMW_RET_ERROR;
    }
    
    ctx->common.graph_cache.clear_on_change_callback();
    
    if (nullptr != ctx->common.graph_guard_condition)
    {
        if (RMW_RET_OK !=
                rmw_connextdds_destroy_guard_condition(
                    ctx->common.graph_guard_condition))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to destroy graph guard condition")
            return RMW_RET_ERROR;
        }
        ctx->common.graph_guard_condition = nullptr;
    }

    if (nullptr != ctx->common.sub)
    {
        if (RMW_RET_OK !=
                rmw_connextdds_destroy_subscriber(ctx, ctx->common.sub))
        {
            RMW_CONNEXT_LOG_ERROR("failed to destroy discovery subscriber")
            return RMW_RET_ERROR;
        }
        ctx->common.sub = nullptr;
    }
    
    if (nullptr != ctx->common.pub)
    {
        if (RMW_RET_OK !=
                rmw_connextdds_destroy_publisher(ctx, ctx->common.pub))
        {
            RMW_CONNEXT_LOG_ERROR("failed to destroy discovery publisher")
            return RMW_RET_ERROR;
        }
        ctx->common.pub = nullptr;
    }

    return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_graph_publish_update(
    rmw_context_impl_t *const ctx,
    void *const msg)
{
    if (nullptr == ctx->common.pub)
    {
        RMW_CONNEXT_LOG_WARNING(
            "context already finalized, message not published")
        return RMW_RET_OK;
    }

    if (RMW_RET_OK != rmw_publish(ctx->common.pub, msg, nullptr))
    {
        RMW_CONNEXT_LOG_ERROR("failed to publish discovery sample")
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_node_created(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    RMW_CONNEXT_LOG_DEBUG_A("[graph] local node created: "
        "node=%s::%s, "
        "dp_gid=%08X.%08X.%08X.%08X",
        node->namespace_,
        node->name,
        ((uint32_t*)ctx->common.gid.data)[0],
        ((uint32_t*)ctx->common.gid.data)[1],
        ((uint32_t*)ctx->common.gid.data)[2],
        ((uint32_t*)ctx->common.gid.data)[3])
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.add_node(
            ctx->common.gid, node->name, node->namespace_);
    
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        static_cast<void>(
            ctx->common.graph_cache.remove_node(
                ctx->common.gid, node->name, node->namespace_));
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_node_deleted(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);

    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.remove_node(
            ctx->common.gid, node->name, node->namespace_);
    
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_graph_on_publisher_created(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Publisher *const pub)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    const rmw_gid_t gid = *pub->gid();
    RMW_CONNEXT_LOG_DEBUG_A("[graph] local publisher created: "
        "node=%s::%s, "
        "dp_gid=%08X.%08X.%08X.%08X, "
        "gid=%08X.%08X.%08X.%08X",
        node->namespace_,
        node->name,
        ((uint32_t*)ctx->common.gid.data)[0],
        ((uint32_t*)ctx->common.gid.data)[1],
        ((uint32_t*)ctx->common.gid.data)[2],
        ((uint32_t*)ctx->common.gid.data)[3],
        ((uint32_t*)gid.data)[0],
        ((uint32_t*)gid.data)[1],
        ((uint32_t*)gid.data)[2],
        ((uint32_t*)gid.data)[3])
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.associate_writer(
            gid,
            ctx->common.gid,
            node->name,
            node->namespace_);
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        static_cast<void>(ctx->common.graph_cache.dissociate_writer(
            gid,
            ctx->common.gid,
            node->name,
            node->namespace_));
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_publisher_deleted(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Publisher *const pub)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
            ctx->common.graph_cache.dissociate_writer(
                *pub->gid(),
                ctx->common.gid,
                node->name,
                node->namespace_);
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_subscriber_created(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Subscriber *const sub)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    const rmw_gid_t gid = *sub->gid();
    RMW_CONNEXT_LOG_DEBUG_A("[graph] local subscriber created: "
        "node=%s::%s, "
        "dp_gid=%08X.%08X.%08X.%08X, "
        "gid=%08X.%08X.%08X.%08X",
        node->namespace_,
        node->name,
        ((uint32_t*)ctx->common.gid.data)[0],
        ((uint32_t*)ctx->common.gid.data)[1],
        ((uint32_t*)ctx->common.gid.data)[2],
        ((uint32_t*)ctx->common.gid.data)[3],
        ((uint32_t*)gid.data)[0],
        ((uint32_t*)gid.data)[1],
        ((uint32_t*)gid.data)[2],
        ((uint32_t*)gid.data)[3])
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.associate_reader(
            gid,
            ctx->common.gid,
            node->name,
            node->namespace_);
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        static_cast<void>(ctx->common.graph_cache.dissociate_reader(
            gid,
            ctx->common.gid,
            node->name,
            node->namespace_));
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_subscriber_deleted(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Subscriber *const sub)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
        rmw_dds_common::msg::ParticipantEntitiesInfo msg =
            ctx->common.graph_cache.dissociate_reader(
                *sub->gid(),
                ctx->common.gid,
                node->name,
                node->namespace_);
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_graph_on_service_created(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Service *const svc)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    const rmw_gid_t pub_gid = *svc->publisher()->gid(),
                    sub_gid = *svc->subscriber()->gid();
    (void)ctx->common.graph_cache.associate_writer(
            pub_gid,
            ctx->common.gid,
            node->name,
            node->namespace_);
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.associate_reader(
            sub_gid,
            ctx->common.gid,
            node->name,
            node->namespace_);
    
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        (void)ctx->common.graph_cache.dissociate_writer(
                pub_gid,
                ctx->common.gid,
                node->name,
                node->namespace_);
        (void)ctx->common.graph_cache.dissociate_reader(
                sub_gid,
                ctx->common.gid,
                node->name,
                node->namespace_);
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_service_deleted(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Service *const svc)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    (void)ctx->common.graph_cache.dissociate_writer(
            *svc->publisher()->gid(),
            ctx->common.gid,
            node->name,
            node->namespace_);
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.dissociate_reader(
            *svc->subscriber()->gid(),
            ctx->common.gid,
            node->name,
            node->namespace_);

    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_client_created(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Client *const client)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    const rmw_gid_t pub_gid = *client->publisher()->gid(),
                    sub_gid = *client->subscriber()->gid();
    (void)ctx->common.graph_cache.associate_writer(
            pub_gid,
            ctx->common.gid,
            node->name,
            node->namespace_);
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.associate_reader(
            sub_gid,
            ctx->common.gid,
            node->name,
            node->namespace_);
    
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        (void)ctx->common.graph_cache.dissociate_writer(
                pub_gid,
                ctx->common.gid,
                node->name,
                node->namespace_);
        (void)ctx->common.graph_cache.dissociate_reader(
                sub_gid,
                ctx->common.gid,
                node->name,
                node->namespace_);
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_client_deleted(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    RMW_Connext_Client *const client)
{
    std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
    (void)ctx->common.graph_cache.dissociate_writer(
            *client->publisher()->gid(),
            ctx->common.gid,
            node->name,
            node->namespace_);
    rmw_dds_common::msg::ParticipantEntitiesInfo msg =
        ctx->common.graph_cache.dissociate_reader(
            *client->subscriber()->gid(),
            ctx->common.gid,
            node->name,
            node->namespace_);
    
    if (RMW_RET_OK !=
            rmw_connextdds_graph_publish_update(ctx, (void*)&msg))
    {
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_participant_info(rmw_context_impl_t * ctx)
{
    RMW_CONNEXT_LOG_DEBUG("ParticipantEntitiesInfo reader active")
    bool taken = false;
    rmw_dds_common::msg::ParticipantEntitiesInfo msg;

    do
    {
        if (RMW_RET_OK != rmw_take(ctx->common.sub, &msg, &taken, nullptr))
        {
            RMW_CONNEXT_LOG_ERROR("failed to take discovery sample")
            return RMW_RET_ERROR;
        }
        if (taken)
        {
            ctx->common.graph_cache.update_participant_entities(msg);
        }
    } while (taken);

    return RMW_RET_OK;
}
