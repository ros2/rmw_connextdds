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

#include <string>

#include "rmw_connextdds/discovery.hpp"
#include "rmw_connextdds/graph_cache.hpp"

// Aliases for rmw.h functions referenced by rmw_dds_common inline functions
#define rmw_publish     rmw_api_connextdds_publish

rmw_ret_t
rmw_connextdds_graph_initialize(rmw_context_impl_t * const ctx)
{
  rmw_qos_profile_t pubsub_qos = rmw_qos_profile_default;
  pubsub_qos.avoid_ros_namespace_conventions = true;
  pubsub_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  pubsub_qos.depth = 5 /* this is quite arbitrary */;
  pubsub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  pubsub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

#if RMW_CONNEXT_HAVE_OPTIONS
  /* Create RMW publisher/subscription/guard condition used by rmw_dds_common
      discovery */
  rmw_publisher_options_t publisher_options =
    rmw_get_default_publisher_options();
  rmw_subscription_options_t subscription_options =
    rmw_get_default_subscription_options();
  subscription_options.ignore_local_publications = true;
#endif /* RMW_CONNEXT_HAVE_OPTIONS */

  const rosidl_message_type_support_t * const type_supports_partinfo =
    rosidl_typesupport_cpp::get_message_type_support_handle<
#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
    rmw_dds_common::msg::ParticipantEntitiesInfo>();
#else
    rmw_connextdds_common::msg::ParticipantEntitiesInfo > ();
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

  const char * const topic_name_partinfo = "ros_discovery_info";

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
#if RMW_CONNEXT_HAVE_OPTIONS
    &publisher_options,
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
    true /* internal */);

  if (nullptr == ctx->common.pub) {
    RMW_CONNEXT_LOG_ERROR(
      "failed to create publisher for ParticipantEntityInfo")
    return RMW_RET_ERROR;
  }


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
#if RMW_CONNEXT_HAVE_OPTIONS
    &subscription_options,
#else
    true /* ignore_local_publications */,
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
    true /* internal */);
  if (nullptr == ctx->common.sub) {
    RMW_CONNEXT_LOG_ERROR(
      "failed to create subscriber for ParticipantEntityInfo")
    return RMW_RET_ERROR;
  }

  RMW_CONNEXT_LOG_DEBUG("configuring discovery")

  ctx->common.graph_guard_condition =
    rmw_connextdds_create_guard_condition();
  if (nullptr == ctx->common.graph_guard_condition) {
    RMW_CONNEXT_LOG_ERROR(
      "failed to create graph guard condition")
    return RMW_RET_BAD_ALLOC;
  }

  ctx->common.graph_cache.set_on_change_callback(
    [gcond = ctx->common.graph_guard_condition]()
    {
      rmw_ret_t ret = rmw_api_connextdds_trigger_guard_condition(gcond);
      if (ret != RMW_RET_OK) {
        RMW_CONNEXT_LOG_ERROR(
          "failed to trigger graph cache on_change_callback")
      }
    });

  rmw_connextdds_get_entity_gid(ctx->participant, ctx->common.gid);

  std::string dp_enclave;

#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
  dp_enclave = ctx->base->options.enclave;
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

  ctx->common.graph_cache.add_participant(ctx->common.gid, dp_enclave);

  if (RMW_RET_OK !=
    rmw_connextdds_dcps_participant_get_reader(
      ctx, &ctx->dr_participants))
  {
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK !=
    rmw_connextdds_dcps_publication_get_reader(
      ctx, &ctx->dr_publications))
  {
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK !=
    rmw_connextdds_dcps_subscription_get_reader(
      ctx, &ctx->dr_subscriptions))
  {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_enable(rmw_context_impl_t * const ctx)
{
  auto pub = reinterpret_cast<RMW_Connext_Publisher *>(ctx->common.pub->data);
  if (RMW_RET_OK != pub->enable()) {
    return RMW_RET_ERROR;
  }

  auto sub = reinterpret_cast<RMW_Connext_Subscriber *>(ctx->common.sub->data);
  if (RMW_RET_OK != sub->enable()) {
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK != rmw_connextdds_enable_builtin_readers(ctx)) {
    return RMW_RET_ERROR;
  }

  rmw_ret_t ret = rmw_connextdds_discovery_thread_start(ctx);
  if (RMW_RET_OK != ret) {
    RMW_CONNEXT_LOG_ERROR("failed to start discovery thread")
    return ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_finalize(rmw_context_impl_t * const ctx)
{
  if (RMW_RET_OK != rmw_connextdds_discovery_thread_stop(ctx)) {
    RMW_CONNEXT_LOG_ERROR("failed to stop discovery thread")
    return RMW_RET_ERROR;
  }

  ctx->common.graph_cache.clear_on_change_callback();

  if (nullptr != ctx->common.graph_guard_condition) {
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

  if (nullptr != ctx->common.sub) {
    if (RMW_RET_OK !=
      rmw_connextdds_destroy_subscriber(ctx, ctx->common.sub))
    {
      RMW_CONNEXT_LOG_ERROR("failed to destroy discovery subscriber")
      return RMW_RET_ERROR;
    }
    ctx->common.sub = nullptr;
  }

  if (nullptr != ctx->common.pub) {
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
  rmw_context_impl_t * const ctx,
  void * const msg)
{
  if (nullptr == ctx->common.pub) {
    RMW_CONNEXT_LOG_WARNING(
      "context already finalized, message not published")
    return RMW_RET_OK;
  }

  if (RMW_RET_OK != rmw_publish(ctx->common.pub, msg, nullptr)) {
    RMW_CONNEXT_LOG_ERROR("failed to publish discovery sample")
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_node_created(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node)
{
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);

  RMW_CONNEXT_LOG_DEBUG_A(
    "[graph] local node created: "
    "node=%s::%s, "
    "dp_gid=%08X.%08X.%08X.%08X",
    node->namespace_,
    node->name,
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[0],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[1],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[2],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[3])
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    ctx->common.graph_cache.add_node(
    ctx->common.gid, node->name, node->namespace_);

  if (RMW_RET_OK !=
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
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
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node)
{
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    ctx->common.graph_cache.remove_node(
    ctx->common.gid, node->name, node->namespace_);

  if (RMW_RET_OK !=
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
  {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_graph_on_publisher_created(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Publisher * const pub)
{
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  const rmw_gid_t gid = *pub->gid();
  RMW_CONNEXT_LOG_DEBUG_A(
    "[graph] local publisher created: "
    "node=%s::%s, "
    "dp_gid=%08X.%08X.%08X.%08X, "
    "gid=%08X.%08X.%08X.%08X",
    node->namespace_,
    node->name,
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[0],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[1],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[2],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[3],
    reinterpret_cast<const uint32_t *>(gid.data)[0],
    reinterpret_cast<const uint32_t *>(gid.data)[1],
    reinterpret_cast<const uint32_t *>(gid.data)[2],
    reinterpret_cast<const uint32_t *>(gid.data)[3])
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    ctx->common.graph_cache.associate_writer(
    gid,
    ctx->common.gid,
    node->name,
    node->namespace_);
  if (RMW_RET_OK !=
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
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
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Publisher * const pub)
{
  rmw_ret_t rc = RMW_RET_ERROR;
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    ctx->common.graph_cache.dissociate_writer(
    *pub->gid(),
    ctx->common.gid,
    node->name,
    node->namespace_);
  rc = rmw_connextdds_graph_publish_update(ctx, reinterpret_cast<void *>(&msg));
  if (RMW_RET_OK != rc)
  {
    return rc;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_subscriber_created(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Subscriber * const sub)
{
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  const rmw_gid_t gid = *sub->gid();
  RMW_CONNEXT_LOG_DEBUG_A(
    "[graph] local subscriber created: "
    "node=%s::%s, "
    "dp_gid=%08X.%08X.%08X.%08X, "
    "gid=%08X.%08X.%08X.%08X",
    node->namespace_,
    node->name,
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[0],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[1],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[2],
    reinterpret_cast<const uint32_t *>(ctx->common.gid.data)[3],
    reinterpret_cast<const uint32_t *>(gid.data)[0],
    reinterpret_cast<const uint32_t *>(gid.data)[1],
    reinterpret_cast<const uint32_t *>(gid.data)[2],
    reinterpret_cast<const uint32_t *>(gid.data)[3])
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    ctx->common.graph_cache.associate_reader(
    gid,
    ctx->common.gid,
    node->name,
    node->namespace_);
  if (RMW_RET_OK !=
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
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
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Subscriber * const sub)
{
  rmw_ret_t rc = RMW_RET_ERROR;
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    ctx->common.graph_cache.dissociate_reader(
    *sub->gid(),
    ctx->common.gid,
    node->name,
    node->namespace_);
  rc = rmw_connextdds_graph_publish_update(ctx, reinterpret_cast<void *>(&msg));
  if (RMW_RET_OK != rc)
  {
    return rc;
  }
  return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_graph_on_service_created(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Service * const svc)
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
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
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
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Service * const svc)
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
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
  {
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_on_client_created(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Client * const client)
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
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
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
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  RMW_Connext_Client * const client)
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
    rmw_connextdds_graph_publish_update(
      ctx, reinterpret_cast<void *>(&msg)))
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

  do {
    if (RMW_RET_OK != rmw_api_connextdds_take(ctx->common.sub, &msg, &taken, nullptr)) {
      RMW_CONNEXT_LOG_ERROR("failed to take discovery sample")
      return RMW_RET_ERROR;
    }
    if (taken) {
      std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
      ctx->common.graph_cache.update_participant_entities(msg);
    }
  } while (taken);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_add_participant(
  rmw_context_impl_t * const ctx,
  const DDS_ParticipantBuiltinTopicData * const data,
  const char * const enclave)
{
  DDS_GUID_t dp_guid;
  rmw_gid_t gid;
  rmw_connextdds_builtinkey_to_guid(&data->key, &dp_guid);
  rmw_connextdds_guid_to_gid(dp_guid, gid);

  if (0 == memcmp(gid.data, ctx->common.gid.data, RMW_GID_STORAGE_SIZE)) {
    /* Ignore own announcements */
    RMW_CONNEXT_LOG_DEBUG(
      "[discovery thread] ignored own participant data")
    return RMW_RET_OK;
  }

  std::string enclave_str;

  if (nullptr != enclave) {
    enclave_str = enclave;
  }

  RMW_CONNEXT_LOG_DEBUG_A(
    "[discovery thread] assert participant: "
    "gid=0x%08X.0x%08X.0x%08X.0x%08X",
    reinterpret_cast<const uint32_t *>(dp_guid.value)[0],
    reinterpret_cast<const uint32_t *>(dp_guid.value)[1],
    reinterpret_cast<const uint32_t *>(dp_guid.value)[2],
    reinterpret_cast<const uint32_t *>(dp_guid.value)[3])

  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  ctx->common.graph_cache.add_participant(gid, enclave_str);

  return RMW_RET_OK;
}

void
rmw_connextdds_graph_add_entity(
  rmw_context_impl_t * ctx,
  const DDS_GUID_t * const endp_guid,
  const DDS_GUID_t * const dp_guid,
  const char * const topic_name,
  const char * const type_name,
  const DDS_ReliabilityQosPolicy * const reliability,
  const DDS_DurabilityQosPolicy * const durability,
  const DDS_DeadlineQosPolicy * const deadline,
  const DDS_LivelinessQosPolicy * const liveliness,
  const bool is_reader)
{
  rmw_gid_t gid;
  rmw_gid_t dp_gid;
  rmw_connextdds_guid_to_gid(*endp_guid, gid);
  rmw_connextdds_guid_to_gid(*dp_guid, dp_gid);

  if (0 == memcmp(dp_gid.data, ctx->common.gid.data, RMW_GID_STORAGE_SIZE)) {
    /* Ignore own announcements */
    return;
  }

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

  if (RMW_RET_OK !=
    rmw_connextdds_readerwriter_qos_to_ros(
      nullptr /* history */,
      reliability,
      durability,
      deadline,
      liveliness,
      &qos_profile))
  {
    // this should never happen with a valid DDS implementation
    RMW_CONNEXT_LOG_ERROR("failed to convert reader qos to ros")
  }

  RMW_CONNEXT_LOG_DEBUG_A(
    "[discovery thread] assert endpoint: "
    "dp_gid=0x%08X.0x%08X.0x%08X.0x%08X, "
    "gid=0x%08X.0x%08X.0x%08X.0x%08X, "
    "topic=%s, "
    "type=%s, "
    "reader=%d",
    reinterpret_cast<const uint32_t *>(dp_guid->value)[0],
    reinterpret_cast<const uint32_t *>(dp_guid->value)[1],
    reinterpret_cast<const uint32_t *>(dp_guid->value)[2],
    reinterpret_cast<const uint32_t *>(dp_guid->value)[3],
    reinterpret_cast<const uint32_t *>(endp_guid->value)[0],
    reinterpret_cast<const uint32_t *>(endp_guid->value)[1],
    reinterpret_cast<const uint32_t *>(endp_guid->value)[2],
    reinterpret_cast<const uint32_t *>(endp_guid->value)[3],
    topic_name,
    type_name,
    is_reader)

  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  ctx->common.graph_cache.add_entity(
    gid,
    std::string(topic_name),
    std::string(type_name),
    dp_gid,
    qos_profile,
    is_reader);
}


rmw_ret_t
rmw_connextdds_graph_remove_participant(
  rmw_context_impl_t * const ctx,
  const DDS_InstanceHandle_t * const instance)
{
  rmw_gid_t dp_gid;
  rmw_connextdds_ih_to_gid(*instance, dp_gid);
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  ctx->common.graph_cache.remove_participant(dp_gid);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_graph_remove_entity(
  rmw_context_impl_t * const ctx,
  const DDS_InstanceHandle_t * const instance,
  const bool is_reader)
{
  rmw_gid_t endp_gid;
  rmw_connextdds_ih_to_gid(*instance, endp_gid);
  std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
  if (!ctx->common.graph_cache.remove_entity(endp_gid, is_reader))
  {
    RMW_CONNEXT_LOG_ERROR("failed to remove entity from cache")
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}
