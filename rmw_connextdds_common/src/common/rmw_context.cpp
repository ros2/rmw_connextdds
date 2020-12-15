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

#include <functional>
#include <memory>
#include <string>

#include "rmw_connextdds/rmw_impl.hpp"
#include "rmw_connextdds/discovery.hpp"
#include "rmw_connextdds/graph_cache.hpp"

#include "rcutils/get_env.h"
#include "rcutils/filesystem.h"

/******************************************************************************
 * Context Implementation
 ******************************************************************************/

rmw_ret_t
rmw_connextdds_initialize_participant_factory_qos(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantFactory * const factory)
{
  UNUSED_ARG(ctx);

  DDS_DomainParticipantFactoryQos qos =
    DDS_DomainParticipantFactoryQos_INITIALIZER;

  if (DDS_RETCODE_OK !=
    DDS_DomainParticipantFactory_get_qos(factory, &qos))
  {
    RMW_CONNEXT_LOG_ERROR("failed to get participant factory qos")
    return RMW_RET_ERROR;
  }

  qos.entity_factory.autoenable_created_entities = DDS_BOOLEAN_FALSE;

  if (DDS_RETCODE_OK !=
    DDS_DomainParticipantFactory_set_qos(factory, &qos))
  {
    RMW_CONNEXT_LOG_ERROR("failed to get participant factory qos")
    DDS_DomainParticipantFactoryQos_finalize(&qos);
    return RMW_RET_ERROR;
  }

  DDS_DomainParticipantFactoryQos_finalize(&qos);
  return RMW_RET_OK;
}

static
rmw_ret_t
rmw_connextdds_initialize_participant_qos(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantQos & dp_qos)
{
  if (DDS_RETCODE_OK !=
    DDS_DomainParticipantFactory_get_default_participant_qos(
      ctx->factory, &dp_qos))
  {
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK !=
    rmw_connextdds_initialize_participant_qos_impl(ctx, &dp_qos))
  {
    return RMW_RET_ERROR;
  }

  /* Lookup and configure initial peer from environment */
  const char * initial_peer = nullptr;
  const char * lookup_rc =
    rcutils_get_env(RMW_CONNEXT_ENV_INITIAL_PEER, &initial_peer);

  if (nullptr != lookup_rc || nullptr == initial_peer) {
    RMW_CONNEXT_LOG_ERROR_A(
      "failed to lookup from environment: "
      "var=%s, "
      "rc=%s ",
      RMW_CONNEXT_ENV_INITIAL_PEER,
      lookup_rc)
    return RMW_RET_ERROR;
  }

  if (strlen(initial_peer) > 0) {
    char * peer = DDS_String_dup(initial_peer);
    if (nullptr == peer) {
      RMW_CONNEXT_LOG_ERROR("failed to allocate peer name")
      return RMW_RET_ERROR;
    }

    if (!DDS_StringSeq_set_maximum(&dp_qos.discovery.initial_peers, 1)) {
      RMW_CONNEXT_LOG_ERROR("failed to set initial peers maximum")
      return RMW_RET_ERROR;
    }
    if (!DDS_StringSeq_set_length(&dp_qos.discovery.initial_peers, 1)) {
      RMW_CONNEXT_LOG_ERROR("failed to set initial peers length");
      return RMW_RET_ERROR;
    }
    *DDS_StringSeq_get_reference(&dp_qos.discovery.initial_peers, 0) =
      peer;

    RMW_CONNEXT_LOG_DEBUG_A("initial peer: %s", peer)
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_impl_t::initialize_node(
  const char * const node_name,
  const char * const node_namespace,
  const bool localhost_only)
{
  RMW_CONNEXT_LOG_DEBUG_A(
    "initializing new node: total=%lu, localhost=%d",
    this->node_count, localhost_only)

  std::lock_guard<std::mutex> guard(this->initialization_mutex);
  if (0u != this->node_count) {
    if ((this->localhost_only && !localhost_only) ||
      (!this->localhost_only && localhost_only))
    {
      RMW_CONNEXT_LOG_ERROR_A(
        "incompatible node for context:"
        "ctx.localhost_only=%d, node.localhost_only=%d",
        this->localhost_only, localhost_only)
      return RMW_RET_ERROR;
    }

    this->node_count += 1;
    RMW_CONNEXT_LOG_DEBUG_A(
      "initialized new node: total=%lu", this->node_count)
    return RMW_RET_OK;
  }

  RMW_CONNEXT_LOG_DEBUG("initializing RMW context")

  this->localhost_only = localhost_only;

  std::ostringstream ss;
  ss << node_namespace;
  this->qos_ctx_namespace = ss.str();
  ss.str("");
  ss << node_name;
  this->qos_ctx_name = ss.str();

  /* Lookup name of custom QoS library */
  const char * qos_library = nullptr;
  const char * lookup_rc =
    rcutils_get_env(RMW_CONNEXT_ENV_QOS_LIBRARY, &qos_library);

  if (nullptr != lookup_rc || nullptr == qos_library) {
    RMW_CONNEXT_LOG_ERROR_A(
      "failed to lookup from environment: "
      "var=%s, "
      "rc=%s ",
      RMW_CONNEXT_ENV_QOS_LIBRARY,
      lookup_rc)
    return RMW_RET_ERROR;
  }

  this->qos_library = qos_library;

  if (RMW_RET_OK != rmw_connextdds_initialize_participant_factory(this)) {
    RMW_CONNEXT_LOG_ERROR(
      "failed to initialize DDS DomainParticipantFactory")
    // no need to call this->clean_up()
    return RMW_RET_ERROR;
  }

  DDS_DomainId_t domain_id =
    static_cast<DDS_DomainId_t>(this->domain_id);

  struct DDS_DomainParticipantQos dp_qos =
    DDS_DomainParticipantQos_INITIALIZER;

  std::unique_ptr<DDS_DomainParticipantQos, std::function<void(DDS_DomainParticipantQos *)>>
  dp_qos_guard(&dp_qos, &DDS_DomainParticipantQos_finalize);
  if (nullptr == dp_qos_guard) {
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK !=
    rmw_connextdds_initialize_participant_qos(this, dp_qos))
  {
    RMW_CONNEXT_LOG_ERROR("failed to initialize participant qos")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK != rmw_connextdds_configure_security(this, &dp_qos)) {
    RMW_CONNEXT_LOG_ERROR("failed to configure DDS Security")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  RMW_CONNEXT_LOG_DEBUG_A(
    "creating DomainParticipant: "
    "domain=%d", domain_id)

  this->participant = DDS_DomainParticipantFactory_create_participant(
    this->factory,
    domain_id,
    &dp_qos,
    nullptr,
    DDS_STATUS_MASK_NONE);
  if (nullptr == this->participant) {
    RMW_CONNEXT_LOG_ERROR("failed to create DDS participant")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  /* Create DDS publisher/subscriber objects that will be used for all DDS writers/readers
      to be created for RMW publishers/subscriptions. */

  DDS_PublisherQos pub_qos = DDS_PublisherQos_INITIALIZER;

  std::unique_ptr<DDS_PublisherQos, std::function<void(DDS_PublisherQos *)>>
  pub_qos_guard(&pub_qos, &DDS_PublisherQos_finalize);
  if (nullptr == pub_qos_guard) {
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_DomainParticipant_get_default_publisher_qos(
      this->participant, &pub_qos))
  {
    RMW_CONNEXT_LOG_ERROR("failed to get default Publisher QoS")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  pub_qos.entity_factory.autoenable_created_entities = DDS_BOOLEAN_FALSE;

  RMW_CONNEXT_LOG_DEBUG("creating default DDS Publisher")

  this->dds_pub = DDS_DomainParticipant_create_publisher(
    this->participant,
    &pub_qos,
    NULL,
    DDS_STATUS_MASK_NONE);

  if (nullptr == this->dds_pub) {
    RMW_CONNEXT_LOG_ERROR("failed to create DDS publisher")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  DDS_SubscriberQos sub_qos = DDS_SubscriberQos_INITIALIZER;

  std::unique_ptr<DDS_SubscriberQos, std::function<void(DDS_SubscriberQos *)>>
  sub_qos_guard(&sub_qos, &DDS_SubscriberQos_finalize);
  if (nullptr == sub_qos_guard) {
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_DomainParticipant_get_default_subscriber_qos(
      this->participant, &sub_qos))
  {
    RMW_CONNEXT_LOG_ERROR("failed to get default Subscriber QoS")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  sub_qos.entity_factory.autoenable_created_entities = DDS_BOOLEAN_FALSE;

  RMW_CONNEXT_LOG_DEBUG("creating default DDS Subscriber")

  this->dds_sub = DDS_DomainParticipant_create_subscriber(
    this->participant,
    &sub_qos,
    NULL,
    DDS_STATUS_MASK_NONE);

  if (nullptr == this->dds_sub) {
    RMW_CONNEXT_LOG_ERROR("failed to create DDS subscriber")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK != rmw_connextdds_graph_initialize(this)) {
    RMW_CONNEXT_LOG_ERROR("failed to initialize graph cache")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  this->node_count = 1;

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(
      DDS_DomainParticipant_as_entity(this->participant)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable participant")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(DDS_Subscriber_as_entity(this->dds_sub)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable dds subscriber")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(DDS_Publisher_as_entity(this->dds_pub)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable dds subscriber")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  if (RMW_RET_OK != rmw_connextdds_graph_enable(this)) {
    RMW_CONNEXT_LOG_ERROR("failed to enable graph cache")
    this->clean_up();
    return RMW_RET_ERROR;
  }

  RMW_CONNEXT_LOG_DEBUG("RMW context initialized")

  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_impl_t::clean_up()
{
  RMW_CONNEXT_LOG_DEBUG("cleaning up RMW context")

  if (RMW_RET_OK != rmw_connextdds_graph_finalize(this)) {
    RMW_CONNEXT_LOG_ERROR("failed to finalize graph cache")
    return RMW_RET_ERROR;
  }

  if (nullptr != this->dds_pub) {
    if (DDS_RETCODE_OK !=
      DDS_DomainParticipant_delete_publisher(
        this->participant, this->dds_pub))
    {
      RMW_CONNEXT_LOG_ERROR("failed to delete DDS publisher")
      return RMW_RET_ERROR;
    }
    this->dds_pub = nullptr;
  }

  if (nullptr != this->dds_sub) {
    if (DDS_RETCODE_OK !=
      DDS_DomainParticipant_delete_subscriber(
        this->participant, this->dds_sub))
    {
      RMW_CONNEXT_LOG_ERROR("failed to delete DDS subscriber")
      return RMW_RET_ERROR;
    }
    this->dds_sub = nullptr;
  }

  if (nullptr != this->participant) {
    if (DDS_RETCODE_OK !=
      DDS_DomainParticipantFactory_delete_participant(
        this->factory, this->participant))
    {
      RMW_CONNEXT_LOG_ERROR("failed to delete DDS participant")
      return RMW_RET_ERROR;
    }
    this->participant = nullptr;
  }

  if (nullptr != this->factory) {
    if (RMW_RET_OK != rmw_connextdds_finalize_participant_factory(this)) {
      RMW_CONNEXT_LOG_ERROR("failed to finalize participant factory")
      return RMW_RET_ERROR;
    }

    DDS_DomainParticipantFactory_finalize_instance();
    this->factory = NULL;
  }

  RMW_CONNEXT_LOG_DEBUG("RMW context finalized")

  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_impl_t::finalize_node()
{
  std::lock_guard<std::mutex> guard(initialization_mutex);
  RMW_CONNEXT_LOG_DEBUG_A(
    "finalizing node: total=%lu", this->node_count)
  this->node_count -= 1;
  if (0u != this->node_count) {
    // destruction shouldn't happen yet
    RMW_CONNEXT_LOG_DEBUG_A(
      "finalized node: total=%lu", this->node_count)
    return RMW_RET_OK;
  }
  RMW_CONNEXT_LOG_DEBUG("all nodes finalized")
  return this->clean_up();
}

uint32_t
rmw_context_impl_t::next_client_id()
{
  std::lock_guard<std::mutex> guard(this->initialization_mutex);
  const uint32_t res = this->client_service_id;
  this->client_service_id += 1;
  if (0 == this->client_service_id) {
    RMW_CONNEXT_LOG_WARNING("rollover detected in client IDs")
  }
  return res;
}

rmw_ret_t
rmw_context_impl_t::assert_topic(
  DDS_DomainParticipant * const participant,
  const char * const topic_name,
  const char * const type_name,
  const bool internal,
  DDS_Topic ** const topic,
  bool & created)
{
  DDS_TopicDescription * const topic_existing =
    DDS_DomainParticipant_lookup_topicdescription(participant, topic_name);

  if (nullptr != topic_existing) {
    if (internal) {
      /* topics for "internal" endpoints are created while the participant
         is still disabled, so find_topic() cannot be called.
         Instead, we cast the topicdescription to a topic. The deletion
         path will have to keep track that the endpoint didn't create
         its topic, so not to try to delete it */
      *topic = DDS_Topic_narrow(topic_existing);
      created = false;
    } else {
      *topic =
        DDS_DomainParticipant_find_topic(
        participant,
        DDS_TopicDescription_get_name(topic_existing),
        &DDS_DURATION_ZERO);
      if (nullptr == *topic) {
        RMW_CONNEXT_LOG_ERROR("failed to find topic from description")
        return RMW_RET_ERROR;
      }
      created = true;
    }
    RMW_CONNEXT_LOG_DEBUG_A(
      "found topic: name=%s, type=%s\n",
      topic_name, type_name);
  } else {
    *topic = DDS_DomainParticipant_create_topic(
      participant,
      topic_name,
      type_name,
      &DDS_TOPIC_QOS_DEFAULT,
      nullptr,
      DDS_STATUS_MASK_NONE);
    if (nullptr == *topic) {
      RMW_CONNEXT_LOG_ERROR("failed to create reader's topic")
      return RMW_RET_ERROR;
    }
    created = true;
    RMW_CONNEXT_LOG_DEBUG_A(
      "created topic: name=%s, type=%s\n",
      topic_name, type_name);
  }

  if (!internal) {
    if (DDS_RETCODE_OK != DDS_Entity_enable(DDS_Topic_as_entity(*topic))) {
      RMW_CONNEXT_LOG_ERROR("failed to enable topic")
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}

/******************************************************************************
 * Context interface functions
 ******************************************************************************/
const char *
rmw_api_connextdds_get_implementation_identifier()
{
  return RMW_CONNEXTDDS_ID;
}


const char *
rmw_api_connextdds_get_serialization_format()
{
  return RMW_CONNEXTDDS_SERIALIZATION_FORMAT;
}


rmw_ret_t
rmw_api_connextdds_set_log_severity(rmw_log_severity_t severity)
{
  return rmw_connextdds_set_log_verbosity(severity);
}


rmw_ret_t
rmw_api_connextdds_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (NULL != init_options->implementation_identifier) {
    RMW_CONNEXT_LOG_ERROR("expected zero-initialized init_options")
    return RMW_RET_INVALID_ARGUMENT;
  }
  init_options->instance_id = 0;
  init_options->implementation_identifier = RMW_CONNEXTDDS_ID;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
#if RMW_CONNEXT_HAVE_LOCALHOST_ONLY
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;
#endif /* RMW_CONNEXT_HAVE_LOCALHOST_ONLY */
#if RMW_CONNEXT_HAVE_OPTIONS
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  init_options->enclave = nullptr;
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
#if RMW_CONNEXT_HAVE_SECURITY
  init_options->security_options = rmw_get_zero_initialized_security_options();
#endif /* RMW_CONNEXT_HAVE_SECURITY */
  return RMW_RET_OK;
}


rmw_ret_t
rmw_api_connextdds_init_options_copy(
  const rmw_init_options_t * src,
  rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  if (nullptr == src->implementation_identifier) {
    RMW_CONNEXT_LOG_ERROR("expected initialized src")
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (nullptr != dst->implementation_identifier) {
    RMW_CONNEXT_LOG_ERROR("expected zero-initialized dst")
    return RMW_RET_INVALID_ARGUMENT;
  }

  rmw_init_options_t tmp = *src;

#if RMW_CONNEXT_HAVE_OPTIONS || RMW_CONNEXT_HAVE_SECURITY
#if RMW_CONNEXT_HAVE_OPTIONS
  const rcutils_allocator_t * allocator = &src->allocator;
  tmp.enclave = rcutils_strdup(tmp.enclave, *allocator);
  if (nullptr != src->enclave && nullptr == tmp.enclave) {
    return RMW_RET_BAD_ALLOC;
  }
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
#if RMW_CONNEXT_HAVE_SECURITY
  tmp.security_options = rmw_get_zero_initialized_security_options();
  rmw_ret_t ret =
    rmw_security_options_copy(&src->security_options, allocator, &tmp.security_options);
  if (RMW_RET_OK != ret) {
    allocator->deallocate(tmp.enclave, allocator->state);
    return ret;
  }
#endif /* RMW_CONNEXT_HAVE_SECURITY */
#endif /* RMW_CONNEXT_HAVE_OPTIONS || RMW_CONNEXT_HAVE_SECURITY */
  *dst = tmp;
  return RMW_RET_OK;
}



rmw_ret_t
rmw_api_connextdds_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  if (nullptr == init_options->implementation_identifier) {
    RMW_CONNEXT_LOG_ERROR("expected initialized init_options")
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

#if RMW_CONNEXT_HAVE_OPTIONS || RMW_CONNEXT_HAVE_SECURITY
  rcutils_allocator_t * allocator = &init_options->allocator;
  RCUTILS_CHECK_ALLOCATOR(allocator, return RMW_RET_INVALID_ARGUMENT);
#if RMW_CONNEXT_HAVE_OPTIONS
  allocator->deallocate(init_options->enclave, allocator->state);
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
#if RMW_CONNEXT_HAVE_SECURITY
  rmw_ret_t ret = rmw_security_options_fini(&init_options->security_options, allocator);
#endif /* RMW_CONNEXT_HAVE_SECURITY */
#else
  rmw_ret_t ret = RMW_RET_OK;
#endif /* RMW_CONNEXT_HAVE_OPTIONS || RMW_CONNEXT_HAVE_SECURITY */
  *init_options = rmw_get_zero_initialized_init_options();
  return ret;
}



rmw_ret_t
rmw_api_connextdds_init(
  const rmw_init_options_t * options,
  rmw_context_t * context)
{
  rmw_ret_t ret = RMW_RET_OK;

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
#if RMW_CONNEXT_HAVE_OPTIONS
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  if (nullptr != context->implementation_identifier) {
    RMW_CONNEXT_LOG_ERROR("expected a zero-initialized context")
    return RMW_RET_INVALID_ARGUMENT;
  }

#if RMW_CONNEXT_HAVE_OPTIONS
  if (options->domain_id >= UINT32_MAX &&
    options->domain_id != RMW_DEFAULT_DOMAIN_ID)
  {
    RMW_CONNEXT_LOG_ERROR("domain id out of range")
    return RMW_RET_INVALID_ARGUMENT;
  }
#endif /* RMW_CONNEXT_HAVE_OPTIONS */

  auto scope_exit_context_reset = rcpputils::make_scope_exit(
    [context]()
    {
      *context = rmw_get_zero_initialized_context();
    });

  context->instance_id = options->instance_id;
  context->implementation_identifier = RMW_CONNEXTDDS_ID;
  const DDS_DomainId_t actual_domain_id =
#if !RMW_CONNEXT_HAVE_OPTIONS
    RMW_CONNEXT_DEFAULT_DOMAIN;
#else
    (RMW_DEFAULT_DOMAIN_ID != options->domain_id) ?
    options->domain_id : RMW_CONNEXT_DEFAULT_DOMAIN;
  ret = rmw_api_connextdds_init_options_copy(options, &context->options);
  if (RMW_RET_OK != ret) {
    RMW_CONNEXT_LOG_ERROR(
      "failed to allocate RMW context implementation")
    return ret;
  }

  auto scope_exit_context_finalize =
    rcpputils::make_scope_exit(
    [context]()
    {
      if (RMW_RET_OK != rmw_api_connextdds_init_options_fini(&context->options)) {
        RMW_CONNEXT_LOG_ERROR(
          "failed to finalize init options")
      }
    });

#endif /* RMW_CONNEXT_HAVE_OPTIONS*/

  /* The context object will be initialized upon creation of the first node */
  context->impl = new (std::nothrow) rmw_context_impl_t(context);
  if (nullptr == context->impl) {
    RMW_CONNEXT_LOG_ERROR(
      "failed to allocate RMW context implementation")
    return RMW_RET_ERROR;
  }

  context->impl->domain_id = actual_domain_id;

#if RMW_CONNEXT_HAVE_OPTIONS
  scope_exit_context_finalize.cancel();
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  scope_exit_context_reset.cancel();
  return ret;
}


rmw_ret_t
rmw_api_connextdds_shutdown(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  context->impl->is_shutdown = true;
  return RMW_RET_OK;
}


rmw_ret_t
rmw_api_connextdds_context_fini(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (!context->impl->is_shutdown) {
    RMW_CONNEXT_LOG_ERROR("context has not been shutdown")
    return RMW_RET_INVALID_ARGUMENT;
  }
#if RMW_CONNEXT_HAVE_OPTIONS
  rmw_ret_t ret = rmw_api_connextdds_init_options_fini(&context->options);
  if (RMW_RET_OK != ret) {
    return ret;
  }
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  delete context->impl;
  *context = rmw_get_zero_initialized_context();
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_configure_security(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantQos * const qos)
{
#if RMW_CONNEXT_HAVE_SECURITY
  if (nullptr == ctx->base->options.security_options.security_root_path) {
    // Security not enabled;
    return RMW_RET_OK;
  }

  rmw_ret_t rc = rmw_connextdds_enable_security(ctx, qos);
  if (RMW_RET_OK != rc) {
    return rc;
  }

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_allocator_t * const allocator_ptr = &allocator;
  std::ostringstream ss;
  std::string prop_uri;
  static const char * const uri_prefix = "file:";

  char * const prop_identity_ca =
    rcutils_join_path(
    ctx->base->options.security_options.security_root_path,
    "identity_ca.cert.pem",
    allocator);
  auto scope_exit_prop_identity_ca = rcpputils::make_scope_exit(
    [allocator_ptr, prop_identity_ca]()
    {
      allocator_ptr->deallocate(prop_identity_ca, allocator_ptr->state);
    });
  ss << uri_prefix << prop_identity_ca;
  prop_uri = ss.str();
  ss.str("");
  /* X509 Certificate of the Identity CA */
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      DDS_SECURITY_IDENTITY_CA_PROPERTY,
      prop_uri.c_str(),
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  char * const prop_perm_ca =
    rcutils_join_path(
    ctx->base->options.security_options.security_root_path,
    "permissions_ca.cert.pem",
    allocator);
  auto scope_exit_prop_perm_ca = rcpputils::make_scope_exit(
    [allocator_ptr, prop_perm_ca]()
    {
      allocator_ptr->deallocate(prop_perm_ca, allocator_ptr->state);
    });
  ss << uri_prefix << prop_perm_ca;
  prop_uri = ss.str();
  ss.str("");
  /* X509 Certificate of the Permissions CA */
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      DDS_SECURITY_PERMISSIONS_CA_PROPERTY,
      prop_uri.c_str(),
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  char * const prop_peer_key =
    rcutils_join_path(
    ctx->base->options.security_options.security_root_path,
    "key.pem",
    allocator);
  auto scope_exit_prop_peer_key = rcpputils::make_scope_exit(
    [allocator_ptr, prop_peer_key]()
    {
      allocator_ptr->deallocate(prop_peer_key, allocator_ptr->state);
    });
  ss << uri_prefix << prop_peer_key;
  prop_uri = ss.str();
  ss.str("");
  /* Private Key of the DomainParticipant's identity */
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      DDS_SECURITY_PRIVATE_KEY_PROPERTY,
      prop_uri.c_str(),
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  char * const prop_peer_cert =
    rcutils_join_path(
    ctx->base->options.security_options.security_root_path,
    "cert.pem",
    allocator);
  auto scope_exit_prop_peer_cert = rcpputils::make_scope_exit(
    [allocator_ptr, prop_peer_cert]()
    {
      allocator_ptr->deallocate(prop_peer_cert, allocator_ptr->state);
    });
  ss << uri_prefix << prop_peer_cert;
  prop_uri = ss.str();
  ss.str("");
  /* Public certificate of the DomainParticipant's identity, signed
   * by the Certificate Authority */
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      DDS_SECURITY_IDENTITY_CERTIFICATE_PROPERTY,
      prop_uri.c_str(),
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  char * const prop_governance =
    rcutils_join_path(
    ctx->base->options.security_options.security_root_path,
    "governance.p7s",
    allocator);
  auto scope_exit_prop_governance = rcpputils::make_scope_exit(
    [allocator_ptr, prop_governance]()
    {
      allocator_ptr->deallocate(prop_governance, allocator_ptr->state);
    });
  ss << uri_prefix << prop_governance;
  prop_uri = ss.str();
  ss.str("");
  /* XML file containing domain governance configuration, signed by
   * the Permission CA */
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      DDS_SECURITY_GOVERNANCE_PROPERTY,
      prop_uri.c_str(),
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  char * const prop_permissions =
    rcutils_join_path(
    ctx->base->options.security_options.security_root_path,
    "permissions.p7s",
    allocator);
  auto scope_exit_prop_permissions = rcpputils::make_scope_exit(
    [allocator_ptr, prop_permissions]()
    {
      allocator_ptr->deallocate(prop_permissions, allocator_ptr->state);
    });
  ss << uri_prefix << prop_permissions;
  prop_uri = ss.str();
  ss.str("");
  /* XML file containing domain permissions configuration, signed by
   * the Permission CA */
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      DDS_SECURITY_PERMISSIONS_PROPERTY,
      prop_uri.c_str(),
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }
#else
  // Security not supported by ROS release
  UNUSED_ARG(ctx);
  UNUSED_ARG(qos);
#endif /* RMW_CONNEXT_HAVE_SECURITY */
  return RMW_RET_OK;
}
