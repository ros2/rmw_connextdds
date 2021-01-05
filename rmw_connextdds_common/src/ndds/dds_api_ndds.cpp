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

#include "rmw_connextdds/type_support.hpp"
#include "rmw_connextdds/rmw_impl.hpp"
#include "rmw_connextdds/graph_cache.hpp"

const char * const RMW_CONNEXTDDS_ID = "rmw_connextdds";
const char * const RMW_CONNEXTDDS_SERIALIZATION_FORMAT = "cdr";

rmw_ret_t
rmw_connextdds_set_log_verbosity(rmw_log_severity_t severity)
{
  NDDS_Config_Logger * logger = NDDS_Config_Logger_get_instance();

  NDDS_Config_LogVerbosity verbosity = NDDS_CONFIG_LOG_VERBOSITY_SILENT;

  switch (severity) {
    case RMW_LOG_SEVERITY_DEBUG:
      {
        verbosity = NDDS_CONFIG_LOG_VERBOSITY_STATUS_ALL;
        break;
      }
    case RMW_LOG_SEVERITY_INFO:
      {
        verbosity = NDDS_CONFIG_LOG_VERBOSITY_STATUS_LOCAL;
        break;
      }
    case RMW_LOG_SEVERITY_WARN:
      {
        verbosity = NDDS_CONFIG_LOG_VERBOSITY_WARNING;
        break;
      }
    case RMW_LOG_SEVERITY_ERROR:
    case RMW_LOG_SEVERITY_FATAL:
      {
        verbosity = NDDS_CONFIG_LOG_VERBOSITY_ERROR;
        break;
      }
    default:
      {
        RMW_CONNEXT_LOG_ERROR("invalid log level")
        return RMW_RET_INVALID_ARGUMENT;
      }
  }

  NDDS_Config_Logger_set_verbosity(logger, verbosity);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_initialize_participant_factory(
  rmw_context_impl_t * const ctx)
{
  UNUSED_ARG(ctx);
  DDS_DomainParticipantFactory * factory = NULL;

  RMW_CONNEXT_LOG_DEBUG("initializing DDS DomainParticipantFactory")

  factory = DDS_DomainParticipantFactory_get_instance();
  if (nullptr == factory) {
    RMW_CONNEXT_LOG_ERROR("failed to get DDS participant factory")
    return RMW_RET_ERROR;
  }

  auto scope_exit_factory_finalize = rcpputils::make_scope_exit(
    []() {DDS_DomainParticipantFactory_finalize_instance();});

  if (RMW_RET_OK !=
    rmw_connextdds_initialize_participant_factory_qos(ctx, factory))
  {
    return RMW_RET_ERROR;
  }

  scope_exit_factory_finalize.cancel();

  RMW_CONNEXT_LOG_DEBUG("DDS DomainParticipantFactory initialized")

  ctx->factory = factory;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_finalize_participant_factory(
  rmw_context_impl_t * const ctx)
{
  UNUSED_ARG(ctx);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_initialize_participant_qos_impl(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantQos * const dp_qos)
{
  UNUSED_ARG(ctx);
  UNUSED_ARG(dp_qos);

  // Only propagate type object
  dp_qos->resource_limits.type_code_max_serialized_length = 0;
#if !RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
  dp_qos->resource_limits.type_object_max_serialized_length = 0;
#endif /* !RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

  // In order to reduce the time to cleanup a participant (Node),
  // we use the advice from
  // https://community.rti.com/static/documentation/connext-dds/5.3.1/doc/api/connext_dds/api_cpp/structDDS__DomainParticipantQos.html
  // and reduce the shutdown_cleanup_period to 50 milliseconds.
  dp_qos->database.shutdown_cleanup_period.sec = 0;
  dp_qos->database.shutdown_cleanup_period.nanosec = 50000000;

  if (ctx->localhost_only) {
    if (DDS_RETCODE_OK !=
      DDS_PropertyQosPolicyHelper_assert_property(
        &dp_qos->property,
        "dds.transport.UDPv4.builtin.parent.allow_interfaces",
        "127.0.0.1",
        DDS_BOOLEAN_FALSE /* propagate */))
    {
      RMW_CONNEXT_LOG_ERROR_A(
        "failed to assert property on participant: %s",
        "dds.transport.UDPv4.builtin.parent.allow_interfaces")
      return RMW_RET_ERROR;
    }
  }

#if 0
  // TODO(asorbini) Enable this property after reassessing all implications
  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &dp_qos->property,
      "dds.transport.UDPv4.builtin.ignore_loopback_interface",
      "0",
      DDS_BOOLEAN_FALSE /* propagate */))
  {
    RMW_CONNEXT_LOG_ERROR(
      "failed to assert property on participant: %s",
      "dds.transport.UDPv4.builtin.ignore_loopback_interface")
    return RMW_RET_ERROR;
  }
#endif

#if RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
  const char * const user_data_fmt = "enclave=%s;";

  const int user_data_len =
    std::snprintf(
    nullptr, 0, user_data_fmt, ctx->base->options.enclave) + 1;

  if (!DDS_OctetSeq_ensure_length(
      &dp_qos->user_data.value, user_data_len, user_data_len))
  {
    RMW_CONNEXT_LOG_ERROR("failed to set user_data length")
    return RMW_RET_ERROR;
  }

  char * const user_data_ptr =
    reinterpret_cast<char *>(
    DDS_OctetSeq_get_contiguous_buffer(&dp_qos->user_data.value));

  const int user_data_rc =
    std::snprintf(
    user_data_ptr,
    user_data_len,
    user_data_fmt,
    ctx->base->options.enclave);

  if (user_data_rc < 0 || user_data_rc != user_data_len - 1) {
    RMW_CONNEXT_LOG_ERROR("failed to set user_data")
    return RMW_RET_ERROR;
  }
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

  // According to the RTPS spec, ContentFilterProperty_t has the following fields:
  // -contentFilteredTopicName (max length 256)
  // -relatedTopicName (max length 256)
  // -filterClassName (max length 256)
  // -filterName (DDSSQL)
  // -filterExpression
  // In Connext, contentfilter_property_max_length is sum of lengths of all these fields,
  // which by default is 256.
  // So we set the limit to 1024, to accomodate the complete topic name with namespaces.
  if (dp_qos->resource_limits.contentfilter_property_max_length !=
    DDS_LENGTH_UNLIMITED)
  {
    dp_qos->resource_limits.contentfilter_property_max_length = 1024;
  }

  // Configure enabled built-in transports
  dp_qos->transport_builtin.mask = DDS_TRANSPORTBUILTIN_UDPv4;
#if RMW_CONNEXT_TRANSPORT_SHMEM
  dp_qos->transport_builtin.mask |= DDS_TRANSPORTBUILTIN_SHMEM;
#endif /* RMW_CONNEXT_TRANSPORT_SHMEM */

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_create_contentfilteredtopic(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipant * const dp,
  DDS_Topic * const base_topic,
  const char * const cft_name,
  const char * const cft_filter,
  DDS_TopicDescription ** const cft_out)
{
  UNUSED_ARG(ctx);
  RMW_CONNEXT_ASSERT(nullptr != cft_name)
  RMW_CONNEXT_ASSERT(nullptr != cft_filter)

  struct DDS_StringSeq cft_parameters;
  DDS_StringSeq_initialize(&cft_parameters);
  DDS_StringSeq_ensure_length(&cft_parameters, 0, 0);

  *cft_out = nullptr;

  DDS_ContentFilteredTopic * cft_topic =
    DDS_DomainParticipant_create_contentfilteredtopic(
    dp, cft_name, base_topic, cft_filter, &cft_parameters);
  if (nullptr == cft_topic) {
    RMW_CONNEXT_LOG_ERROR_A(
      "failed to create content-filtered topic: "
      "name=%s, filter=%s", cft_name, cft_filter)
    return RMW_RET_ERROR;
  }

  *cft_out = DDS_ContentFilteredTopic_as_topicdescription(cft_topic);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_delete_contentfilteredtopic(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipant * const dp,
  DDS_TopicDescription * const cft_topicdesc)
{
  UNUSED_ARG(ctx);

  DDS_ContentFilteredTopic * const cft_topic =
    DDS_ContentFilteredTopic_narrow(cft_topicdesc);

  if (DDS_RETCODE_OK !=
    DDS_DomainParticipant_delete_contentfilteredtopic(dp, cft_topic))
  {
    RMW_CONNEXT_LOG_ERROR("failed to delete content-filtered topic")
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

static
rmw_ret_t
rmw_connextdds_get_qos_policies(
  const bool writer_qos,
  RMW_Connext_MessageTypeSupport * const type_support,
  DDS_PropertyQosPolicy * const property,
  const rmw_qos_profile_t * const qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
  ,
  const rmw_publisher_options_t * const pub_options,
  const rmw_subscription_options_t * const sub_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
)
{
  UNUSED_ARG(qos_policies);
#if RMW_CONNEXT_HAVE_OPTIONS
  UNUSED_ARG(pub_options);
  UNUSED_ARG(sub_options);
#endif /* RMW_CONNEXT_HAVE_OPTIONS */

  /* if type is unbound set property force allocation of samples from heap */
  if (type_support->unbounded()) {
    const char * property_name = nullptr;
    if (writer_qos) {
      property_name =
        "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size";
    } else {
      property_name =
        "dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size";
    }
    if (DDS_RETCODE_OK !=
      DDS_PropertyQosPolicyHelper_assert_property(
        property, property_name, "0",
        DDS_BOOLEAN_FALSE /* propagate */))
    {
      RMW_CONNEXT_LOG_ERROR("failed to set property qos on writer")
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_get_datawriter_qos(
  rmw_context_impl_t * const ctx,
  RMW_Connext_MessageTypeSupport * const type_support,
  DDS_Topic * const topic,
  DDS_DataWriterQos * const qos,
  const rmw_qos_profile_t * const qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
  ,
  const rmw_publisher_options_t * const pub_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
)
{
  UNUSED_ARG(ctx);
  UNUSED_ARG(topic);

  if (RMW_RET_OK !=
    rmw_connextdds_get_readerwriter_qos(
      true /* writer_qos */,
      type_support,
      &qos->history,
      &qos->reliability,
      &qos->durability,
      &qos->deadline,
      &qos->liveliness,
      &qos->resource_limits,
      // TODO(asorbini) this value is not actually used, remove it
      &qos->publish_mode,
      qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
      ,
      pub_options,
      nullptr           /* sub_options */
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  ))
  {
    return RMW_RET_ERROR;
  }

  qos->publish_mode.kind = DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS;

  return rmw_connextdds_get_qos_policies(
    true /* writer_qos */,
    type_support,
    &qos->property,
    qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
    ,
    pub_options,
    nullptr             /* sub_options */
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  );
}

rmw_ret_t
rmw_connextdds_get_datareader_qos(
  rmw_context_impl_t * const ctx,
  RMW_Connext_MessageTypeSupport * const type_support,
  DDS_TopicDescription * const topic_desc,
  DDS_DataReaderQos * const qos,
  const rmw_qos_profile_t * const qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
  , const rmw_subscription_options_t * const sub_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
)
{
  UNUSED_ARG(ctx);
  UNUSED_ARG(topic_desc);

  if (RMW_RET_OK !=
    rmw_connextdds_get_readerwriter_qos(
      false /* writer_qos */,
      type_support,
      &qos->history,
      &qos->reliability,
      &qos->durability,
      &qos->deadline,
      &qos->liveliness,
      &qos->resource_limits,
      nullptr /* publish_mode */,
      qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
      ,
      nullptr /* pub_options */,
      sub_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  ))
  {
    return RMW_RET_ERROR;
  }
  return rmw_connextdds_get_qos_policies(
    false /* writer_qos */,
    type_support,
    &qos->property,
    qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
    ,
    nullptr /* pub_options */,
    sub_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  );
}

DDS_DataWriter *
rmw_connextdds_create_datawriter(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipant * const participant,
  DDS_Publisher * const pub,
  const rmw_qos_profile_t * const qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS
  const rmw_publisher_options_t * const publisher_options,
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  const bool internal,
  RMW_Connext_MessageTypeSupport * const type_support,
  DDS_Topic * const topic,
  DDS_DataWriterQos * const dw_qos)
{
  UNUSED_ARG(ctx);
  UNUSED_ARG(participant);
  UNUSED_ARG(internal);

  if (RMW_RET_OK !=
    rmw_connextdds_get_datawriter_qos(
      ctx, type_support, topic, dw_qos, qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
      , publisher_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  ))
  {
    RMW_CONNEXT_LOG_ERROR("failed to convert writer QoS")
    return nullptr;
  }

  DDS_DataWriter * const writer =
    DDS_Publisher_create_datawriter(
    pub, topic, dw_qos,
    NULL, DDS_STATUS_MASK_NONE);

  return writer;
}

DDS_DataReader *
rmw_connextdds_create_datareader(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipant * const participant,
  DDS_Subscriber * const sub,
  const rmw_qos_profile_t * const qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS
  const rmw_subscription_options_t * const subscriber_options,
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  const bool internal,
  RMW_Connext_MessageTypeSupport * const type_support,
  DDS_TopicDescription * const topic_desc,
  DDS_DataReaderQos * const dr_qos)
{
  UNUSED_ARG(ctx);
  UNUSED_ARG(participant);
  UNUSED_ARG(internal);

  if (RMW_RET_OK !=
    rmw_connextdds_get_datareader_qos(
      ctx, type_support, topic_desc, dr_qos, qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
      , subscriber_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  ))
  {
    RMW_CONNEXT_LOG_ERROR("failed to convert reader QoS")
    return nullptr;
  }

  return DDS_Subscriber_create_datareader(
    sub, topic_desc, dr_qos, NULL, DDS_STATUS_MASK_NONE);
}

rmw_ret_t
rmw_connextdds_write_message(
  RMW_Connext_Publisher * const pub,
  RMW_Connext_Message * const message,
  int64_t * const sn_out)
{
#if !RMW_CONNEXT_EMULATE_REQUESTREPLY
  if (pub->message_type_support()->type_requestreply()) {
    const RMW_Connext_RequestReplyMessage * const rr_msg =
      reinterpret_cast<const RMW_Connext_RequestReplyMessage *>(message->user_data);
    DDS_WriteParams_t write_params = DDS_WRITEPARAMS_DEFAULT;

    if (!rr_msg->request) {
      /* If this is a reply, propagate the request's sample identity
         via the related_sample_identity field */
      rmw_ret_t rc = RMW_RET_ERROR;

      rmw_connextdds_sn_ros_to_dds(
        rr_msg->sn,
        write_params.related_sample_identity.sequence_number);

      rc = rmw_connextdds_gid_to_guid(
        rr_msg->gid,
        write_params.related_sample_identity.writer_guid);
      if (RMW_RET_OK != rc) {
        return rc;
      }
    } else {
      // enable WriteParams::replace_auto to retrieve SN of published message
      write_params.replace_auto = DDS_BOOLEAN_TRUE;
    }

    if (DDS_RETCODE_OK !=
      DDS_DataWriter_write_w_params_untypedI(
        pub->writer(), message, &write_params))
    {
      RMW_CONNEXT_LOG_ERROR(
        "failed to write request/reply message to DDS")
      return RMW_RET_ERROR;
    }

    if (rr_msg->request) {
      int64_t sn = 0;

      // Read assigned sn from write_params
      rmw_connextdds_sn_dds_to_ros(
        write_params.identity.sequence_number, sn);

      *sn_out = sn;
    }

    return RMW_RET_OK;
  }
#else
  UNUSED_ARG(sn_out);
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */

  if (DDS_RETCODE_OK !=
    DDS_DataWriter_write_untypedI(
      pub->writer(), message, &DDS_HANDLE_NIL))
  {
    RMW_CONNEXT_LOG_ERROR("failed to write message to DDS")
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_take_samples(
  RMW_Connext_Subscriber * const sub)
{
  DDS_Boolean is_loan = DDS_BOOLEAN_TRUE;
  DDS_Long data_len = 0;
  void ** data_buffer = nullptr;

  DDS_ReturnCode_t rc =
    DDS_DataReader_read_or_take_instance_untypedI(
    sub->reader(),
    &is_loan,
    &data_buffer,
    &data_len,
    sub->info_seq(),
    0 /* data_seq_len */,
    0 /* data_seq_max_len */,
    DDS_BOOLEAN_TRUE /* data_seq_has_ownership */,
    NULL /* data_seq_contiguous_buffer_for_copy */,
    1 /* data_size -- ignored because loaning*/,
    DDS_LENGTH_UNLIMITED /* max_samples */,
    &DDS_HANDLE_NIL /* a_handle */,
#if !RMW_CONNEXT_DDS_API_PRO_LEGACY
    NULL /* topic_query_guid */,
#endif /* RMW_CONNEXT_DDS_API_PRO_LEGACY */
    DDS_ANY_SAMPLE_STATE,
    DDS_ANY_VIEW_STATE,
    DDS_ANY_INSTANCE_STATE,
    DDS_BOOLEAN_TRUE /* take */);
  if (DDS_RETCODE_OK != rc) {
    if (DDS_RETCODE_NO_DATA == rc) {
      return RMW_RET_OK;
    }
    RMW_CONNEXT_LOG_ERROR("failed to take data from DDS reader")
    return RMW_RET_ERROR;
  }
  RMW_CONNEXT_ASSERT(data_len > 0)(void) RMW_Connext_Uint8ArrayPtrSeq_loan_contiguous(
    sub->data_seq(),
    reinterpret_cast<rcutils_uint8_array_t **>(data_buffer),
    data_len,
    data_len);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_return_samples(
  RMW_Connext_Subscriber * const sub)
{
  void ** data_buffer = reinterpret_cast<void **>(
    RMW_Connext_Uint8ArrayPtrSeq_get_contiguous_buffer(sub->data_seq()));
  const size_t data_len =
    RMW_Connext_Uint8ArrayPtrSeq_get_length(sub->data_seq());

  if (!RMW_Connext_Uint8ArrayPtrSeq_unloan(sub->data_seq())) {
    RMW_CONNEXT_LOG_ERROR("failed to unloan sample sequence")
    return RMW_RET_ERROR;
  }
  if (DDS_RETCODE_OK !=
    DDS_DataReader_return_loan_untypedI(
      sub->reader(),
      data_buffer,
      data_len,
      sub->info_seq()))
  {
    RMW_CONNEXT_LOG_ERROR("failed to return loan to DDS reader")
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_filter_sample(
  RMW_Connext_Subscriber * const sub,
  const void * const sample,
  const DDS_SampleInfo * const info,
  const DDS_InstanceHandle_t * const request_writer_handle,
  bool * const accepted)
{
  UNUSED_ARG(sub);
  UNUSED_ARG(sample);
  UNUSED_ARG(info);

  *accepted = true;

  if (sub->ignore_local()) {
    DDS_InstanceHandle_t reader_ih = sub->participant_instance_handle();

    *accepted = (0 != memcmp(
        reader_ih.keyHash.value,
        info->original_publication_virtual_guid.value,
        12));
  }

  if (!*accepted) {
    return RMW_RET_OK;
  }

  if (nullptr != request_writer_handle) {
    DDS_SampleIdentity_t related_sample_identity;
    DDS_SampleInfo_get_related_sample_identity(
      info, &related_sample_identity);

    // Convert instance handle to guid
    DDS_GUID_t writer_guid = DDS_GUID_DEFAULT;
    memcpy(writer_guid.value, request_writer_handle->keyHash.value, MIG_RTPS_KEY_HASH_MAX_LENGTH);

    *accepted =
      (DDS_GUID_compare(
        &writer_guid, &related_sample_identity.writer_guid) == 0);
  }

  return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_dcps_participant_get_reader(
  rmw_context_impl_t * const ctx,
  DDS_DataReader ** const reader_out)
{
  DDS_Subscriber * const sub =
    DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
  if (nullptr == sub) {
    return RMW_RET_ERROR;
  }

  DDS_DataReader * const reader =
    DDS_Subscriber_lookup_datareader(sub, DDS_PARTICIPANT_TOPIC_NAME);

  if (nullptr == reader) {
    return RMW_RET_ERROR;
  }

  *reader_out = reader;
  return RMW_RET_OK;
}


rmw_ret_t
rmw_connextdds_dcps_publication_get_reader(
  rmw_context_impl_t * const ctx,
  DDS_DataReader ** const reader_out)
{
  DDS_Subscriber * const sub =
    DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
  if (nullptr == sub) {
    return RMW_RET_ERROR;
  }

  DDS_DataReader * const reader =
    DDS_Subscriber_lookup_datareader(sub, DDS_PUBLICATION_TOPIC_NAME);

  if (nullptr == reader) {
    return RMW_RET_ERROR;
  }

  *reader_out = reader;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_subscription_get_reader(
  rmw_context_impl_t * const ctx,
  DDS_DataReader ** const reader_out)
{
  DDS_Subscriber * const sub =
    DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
  if (nullptr == sub) {
    return RMW_RET_ERROR;
  }

  DDS_DataReader * const reader =
    DDS_Subscriber_lookup_datareader(sub, DDS_SUBSCRIPTION_TOPIC_NAME);

  if (nullptr == reader) {
    return RMW_RET_ERROR;
  }

  *reader_out = reader;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_enable_builtin_readers(rmw_context_impl_t * const ctx)
{
  /* Enable builtin subscriber and endpoints */
  DDS_Subscriber * const sub =
    DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
  if (nullptr == sub) {
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK != DDS_Entity_enable(DDS_Subscriber_as_entity(sub))) {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin subscriber")
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(
      DDS_Topic_as_entity(
        DDS_Topic_narrow(
          DDS_DataReader_get_topicdescription(
            ctx->dr_participants)))))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin topic (participants)")
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(DDS_DataReader_as_entity(ctx->dr_participants)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin reader (participants)")
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(
      DDS_Topic_as_entity(
        DDS_Topic_narrow(
          DDS_DataReader_get_topicdescription(
            ctx->dr_publications)))))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin topic (publications)")
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(DDS_DataReader_as_entity(ctx->dr_publications)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin reader (publications)")
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(
      DDS_Topic_as_entity(
        DDS_Topic_narrow(
          DDS_DataReader_get_topicdescription(
            ctx->dr_subscriptions)))))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin topic (subscriptions)")
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_Entity_enable(DDS_DataReader_as_entity(ctx->dr_subscriptions)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to enable builtin reader (subscriptions)")
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_participant_on_data(rmw_context_impl_t * const ctx)
{
  DDS_ParticipantBuiltinTopicDataSeq data_seq = DDS_SEQUENCE_INITIALIZER;
  DDS_SampleInfoSeq info_seq = DDS_SEQUENCE_INITIALIZER;
  DDS_ReturnCode_t rc = DDS_RETCODE_OK;

  DDS_ParticipantBuiltinTopicDataDataReader * const reader =
    DDS_ParticipantBuiltinTopicDataDataReader_narrow(ctx->dr_participants);

  do {
    rc = DDS_ParticipantBuiltinTopicDataDataReader_take(
      reader,
      &data_seq,
      &info_seq,
      DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE,
      DDS_ANY_VIEW_STATE,
      DDS_ANY_INSTANCE_STATE);
    if (DDS_RETCODE_OK != rc) {
      continue;
    }

    const size_t data_len =
      DDS_ParticipantBuiltinTopicDataSeq_get_length(&data_seq);
    for (size_t i = 0; i < data_len; i++) {
      DDS_ParticipantBuiltinTopicData * const data =
        DDS_ParticipantBuiltinTopicDataSeq_get_reference(&data_seq, i);
      DDS_SampleInfo * const info =
        DDS_SampleInfoSeq_get_reference(&info_seq, i);

      if (!info->valid_data) {
        /* TODO(asorbini): Check for instance_state != ALIVE to remove
           the remote participant from the graph_cache by calling:
           graph_cache.remove_participant(gid) */
        if (info->instance_state == DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE ||
          info->instance_state == DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE)
        {
          if (RMW_RET_OK !=
            rmw_connextdds_graph_remove_participant(ctx, &info->instance_handle))
          {
            // TODO(asorbini) log without lock
            continue;
          }
        } else {
          RMW_CONNEXT_LOG_DEBUG(
            "[discovery thread] ignored participant invalid data")
        }

        continue;
      }

      if (RMW_RET_OK != rmw_connextdds_graph_add_participant(ctx, data)) {
        RMW_CONNEXT_LOG_ERROR(
          "failed to asser remote participant in graph")
        continue;
      }
    }

    if (DDS_RETCODE_OK !=
      DDS_ParticipantBuiltinTopicDataDataReader_return_loan(
        reader, &data_seq, &info_seq))
    {
      RMW_CONNEXT_LOG_ERROR("failed to return loan to dds reader")
      return RMW_RET_ERROR;
    }
  } while (DDS_RETCODE_OK == rc);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_publication_on_data(rmw_context_impl_t * const ctx)
{
  DDS_PublicationBuiltinTopicDataSeq data_seq = DDS_SEQUENCE_INITIALIZER;
  DDS_SampleInfoSeq info_seq = DDS_SEQUENCE_INITIALIZER;
  DDS_ReturnCode_t rc = DDS_RETCODE_OK;

  DDS_PublicationBuiltinTopicDataDataReader * const reader =
    DDS_PublicationBuiltinTopicDataDataReader_narrow(ctx->dr_publications);

  do {
    rc = DDS_PublicationBuiltinTopicDataDataReader_take(
      reader,
      &data_seq,
      &info_seq,
      DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE,
      DDS_ANY_VIEW_STATE,
      DDS_ANY_INSTANCE_STATE);
    if (DDS_RETCODE_OK != rc) {
      continue;
    }

    const size_t data_len =
      DDS_PublicationBuiltinTopicDataSeq_get_length(&data_seq);
    for (size_t i = 0; i < data_len; i++) {
      DDS_PublicationBuiltinTopicData * const data =
        DDS_PublicationBuiltinTopicDataSeq_get_reference(&data_seq, i);
      DDS_SampleInfo * const info =
        DDS_SampleInfoSeq_get_reference(&info_seq, i);

      if (!info->valid_data) {
        if (info->instance_state == DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE ||
          info->instance_state == DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE)
        {
          if (RMW_RET_OK !=
            rmw_connextdds_graph_remove_entity(
              ctx, &info->instance_handle, false /* is_reader */))
          {
            // TODO(asorbini) log without lock
            continue;
          }
        } else {
          RMW_CONNEXT_LOG_DEBUG("[discovery thread] ignored publication invalid data")
        }
        continue;
      }

      DDS_GUID_t endp_guid;
      DDS_GUID_t dp_guid;

      DDS_BuiltinTopicKey_to_guid(&data->key, &endp_guid);
      DDS_BuiltinTopicKey_to_guid(&data->participant_key, &dp_guid);

      rmw_connextdds_graph_add_entity(
        ctx,
        &endp_guid,
        &dp_guid,
        data->topic_name,
        data->type_name,
        &data->reliability,
        &data->durability,
        &data->deadline,
        &data->liveliness,
        false /* is_reader */);
    }

    if (DDS_RETCODE_OK !=
      DDS_PublicationBuiltinTopicDataDataReader_return_loan(
        reader, &data_seq, &info_seq))
    {
      RMW_CONNEXT_LOG_ERROR("failed to return loan to dds reader")
      return RMW_RET_ERROR;
    }
  } while (DDS_RETCODE_OK == rc);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_subscription_on_data(rmw_context_impl_t * const ctx)
{
  DDS_SubscriptionBuiltinTopicDataSeq data_seq = DDS_SEQUENCE_INITIALIZER;
  DDS_SampleInfoSeq info_seq = DDS_SEQUENCE_INITIALIZER;
  DDS_ReturnCode_t rc = DDS_RETCODE_OK;

  DDS_SubscriptionBuiltinTopicDataDataReader * const reader =
    DDS_SubscriptionBuiltinTopicDataDataReader_narrow(
    ctx->dr_subscriptions);

  do {
    rc = DDS_SubscriptionBuiltinTopicDataDataReader_take(
      reader,
      &data_seq,
      &info_seq,
      DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE,
      DDS_ANY_VIEW_STATE,
      DDS_ANY_INSTANCE_STATE);
    if (DDS_RETCODE_OK != rc) {
      continue;
    }

    const size_t data_len =
      DDS_SubscriptionBuiltinTopicDataSeq_get_length(&data_seq);
    for (size_t i = 0; i < data_len; i++) {
      DDS_SubscriptionBuiltinTopicData * const data =
        DDS_SubscriptionBuiltinTopicDataSeq_get_reference(&data_seq, i);
      DDS_SampleInfo * const info =
        DDS_SampleInfoSeq_get_reference(&info_seq, i);

      if (!info->valid_data) {
        if (info->instance_state == DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE ||
          info->instance_state == DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE)
        {
          if (RMW_RET_OK !=
            rmw_connextdds_graph_remove_entity(
              ctx, &info->instance_handle, true /* is_reader */))
          {
            // TODO(asorbini) log without lock
            continue;
          }
        } else {
          RMW_CONNEXT_LOG_DEBUG("[discovery thread] ignored subscription invalid data")
        }
        continue;
      }

      DDS_GUID_t endp_guid;
      DDS_GUID_t dp_guid;

      DDS_BuiltinTopicKey_to_guid(&data->key, &endp_guid);
      DDS_BuiltinTopicKey_to_guid(&data->participant_key, &dp_guid);

      rmw_connextdds_graph_add_entity(
        ctx,
        &endp_guid,
        &dp_guid,
        data->topic_name,
        data->type_name,
        &data->reliability,
        &data->durability,
        &data->deadline,
        &data->liveliness,
        true /* is_reader */);
    }

    if (DDS_RETCODE_OK !=
      DDS_SubscriptionBuiltinTopicDataDataReader_return_loan(
        reader, &data_seq, &info_seq))
    {
      RMW_CONNEXT_LOG_ERROR("failed to return loan to dds reader")
      return RMW_RET_ERROR;
    }
  } while (DDS_RETCODE_OK == rc);

  return RMW_RET_OK;
}

void
rmw_connextdds_ih_to_gid(const DDS_InstanceHandle_t & ih, rmw_gid_t & gid)
{
  static_assert(
    RMW_GID_STORAGE_SIZE >= MIG_RTPS_KEY_HASH_MAX_LENGTH,
    "rmw_gid_t type too small for an RTI Connext DDS Micro GUID");

  memset(&gid, 0, sizeof(gid));
  gid.implementation_identifier = RMW_CONNEXTDDS_ID;
  memcpy(gid.data, ih.keyHash.value, MIG_RTPS_KEY_HASH_MAX_LENGTH);
}

void
rmw_connextdds_configure_subscriber_condition_listener(
  RMW_Connext_Subscriber * const sub,
  RMW_Connext_StdSubscriberStatusCondition * cond,
  DDS_DataReaderListener * const listener,
  DDS_StatusMask * const listener_mask)
{
  UNUSED_ARG(sub);
  UNUSED_ARG(cond);
  UNUSED_ARG(listener_mask);
  UNUSED_ARG(listener);
}

void
rmw_connextdds_builtinkey_to_guid(
  const DDS_BuiltinTopicKey_t * const self,
  DDS_GUID_t * const dst)
{
  DDS_BuiltinTopicKey_to_guid(self, dst);
}

rmw_ret_t
rmw_connextdds_enable_security(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipantQos * const qos)
{
  UNUSED_ARG(ctx);

  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      "com.rti.serv.load_plugin",
      "com.rti.serv.secure",
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      "com.rti.serv.secure.library",
      "nddssecurity",
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  if (DDS_RETCODE_OK !=
    DDS_PropertyQosPolicyHelper_assert_property(
      &qos->property,
      "com.rti.serv.secure.create_function",
      "RTI_Security_PluginSuite_create",
      RTI_FALSE))
  {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}
