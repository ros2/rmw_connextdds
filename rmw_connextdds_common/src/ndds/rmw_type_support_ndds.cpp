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

#include "rmw_connextdds/type_support.hpp"
#include "rmw_connextdds/typecode.hpp"
#include "rmw_connextdds/rmw_impl.hpp"

#ifndef cdr_type_object_h
#include "cdr/cdr_typeObject.h"
#endif

#ifndef cdr_log_h
#include "cdr/cdr_log.h"
#endif

/******************************************************************************
 * Connext Pro TypePlugin
 ******************************************************************************/
struct RMW_Connext_NddsTypePluginI;

struct RMW_Connext_NddsTypeCode
{
  RTICdrTypeCode base;
  RMW_Connext_NddsTypePluginI * type_plugin;
  DDS_TypeCode * dds_tc;
  RMW_Connext_TypeCodePtrSeq tc_cache;

  ~RMW_Connext_NddsTypeCode()
  {
    if (nullptr != this->dds_tc) {
      rmw_connextdds_delete_typecode(this->dds_tc);
    }
    rmw_connextdds_release_typecode_cache(&this->tc_cache);
  }

  RMW_Connext_NddsTypeCode(
    RMW_Connext_NddsTypePluginI * const type_plugin,
    DDS_TypeCode * const dds_tc = nullptr,
    RMW_Connext_TypeCodePtrSeq * const tc_cache = nullptr)
  : type_plugin(type_plugin),
    dds_tc(dds_tc)
  {
    if (nullptr != this->dds_tc) {
      this->base = this->dds_tc->_data;
    }
    if (nullptr != tc_cache) {
      this->tc_cache = *tc_cache;
    } else {
      RMW_Connext_TypeCodePtrSeq_initialize(&this->tc_cache);
    }
  }
};

struct RMW_Connext_NddsTypePluginI
{
  struct PRESTypePlugin base;
  RMW_Connext_MessageTypeSupport * wrapper;
  RMW_Connext_NddsTypeCode type_code;
  REDAFastBufferPool * pool_samples;
  uint32_t attached_count;

  RMW_Connext_NddsTypePluginI(
    RMW_Connext_MessageTypeSupport * const wrapper,
    REDAFastBufferPool * const pool_samples,
    DDS_TypeCode * const tc = nullptr,
    RMW_Connext_TypeCodePtrSeq * const tc_cache = nullptr)
  : wrapper(wrapper),
    type_code(this, tc, tc_cache),
    pool_samples(pool_samples),
    attached_count(1)
  {}

  ~RMW_Connext_NddsTypePluginI()
  {
    REDAFastBufferPool_delete(this->pool_samples);
    delete this->wrapper;
  }

  rcutils_uint8_array_t *
  allocate_sample()
  {
    return reinterpret_cast<rcutils_uint8_array_t *>(
      REDAFastBufferPool_getBuffer(this->pool_samples));
  }

  void
  return_sample(rcutils_uint8_array_t * const sample)
  {
    REDAFastBufferPool_returnBuffer(this->pool_samples, sample);
  }
};


struct RMW_Connext_NddsParticipantData
{
  PRESTypePluginDefaultParticipantData pres_data;
  PRESTypePluginDefaultParticipantData * pres_data_ptr;
  RMW_Connext_NddsTypePluginI * type_plugin;

  RMW_Connext_NddsParticipantData(
    PRESTypePluginDefaultParticipantData * const pres_data,
    RMW_Connext_NddsTypePluginI * const type_plugin)
  : pres_data(*pres_data),
    pres_data_ptr(pres_data),
    type_plugin(type_plugin)
  {}

  ~RMW_Connext_NddsParticipantData()
  {
    PRESTypePluginDefaultParticipantData_delete(this->pres_data_ptr);
    delete this->type_plugin;
  }
};


static RTIBool
RMW_Connext_Uint8ArrayPtr_initialize_w_params(
  rcutils_uint8_array_t ** self,
  const struct DDS_TypeAllocationParams_t * allocParams)
{
  UNUSED_ARG(allocParams);
  *self = NULL;
  return RTI_TRUE;
}

static RTIBool
RMW_Connext_Uint8ArrayPtr_finalize_w_params(
  rcutils_uint8_array_t ** self,
  const struct DDS_TypeDeallocationParams_t * deallocParams)
{
  UNUSED_ARG(deallocParams);
  *self = NULL;
  return RTI_TRUE;
}

static RTIBool
RMW_Connext_Uint8ArrayPtr_copy(
  rcutils_uint8_array_t ** dst,
  const rcutils_uint8_array_t ** src)
{
  *dst = const_cast<rcutils_uint8_array_t *>(*src);
  return RTI_TRUE;
}

#define T                       rcutils_uint8_array_t *
#define TSeq                    RMW_Connext_Uint8ArrayPtrSeq
#define T_initialize_w_params   RMW_Connext_Uint8ArrayPtr_initialize_w_params
#define T_finalize_w_params     RMW_Connext_Uint8ArrayPtr_finalize_w_params
#define T_copy                  RMW_Connext_Uint8ArrayPtr_copy
#include "dds_c/generic/dds_c_sequence_TSeq.gen"

static
unsigned int
RMW_Connext_TypePlugin_get_serialized_sample_max_size(
  PRESTypePluginEndpointData endpoint_data,
  RTIBool include_encapsulation,
  RTIEncapsulationId encapsulation_id,
  unsigned int current_alignment);

static
unsigned int
RMW_Connext_TypePlugin_get_serialized_sample_size(
  PRESTypePluginEndpointData endpoint_data,
  RTIBool include_encapsulation,
  RTIEncapsulationId encapsulation_id,
  unsigned int current_alignment,
  const void * sample);

RTIBool
RMW_Connext_TypePlugin_get_buffer(
  PRESTypePluginEndpointData endpointData,
  struct REDABuffer * buffer,
  RTIEncapsulationId encapsulationId,
  const void * user_data)
{
  return PRESTypePluginDefaultEndpointData_getBuffer(
    endpointData, buffer, encapsulationId, user_data);
}

void
RMW_Connext_TypePlugin_return_buffer(
  PRESTypePluginEndpointData endpointData,
  struct REDABuffer * buffer,
  RTIEncapsulationId encapsulationId)
{
  return PRESTypePluginDefaultEndpointData_returnBuffer(
    endpointData, buffer, encapsulationId);
}

void *
RMW_Connext_TypePlugin_get_sample(
  PRESTypePluginEndpointData endpointData,
  void ** handle /* out */)
{
  return PRESTypePluginDefaultEndpointData_getSample(endpointData, handle);
}

void *
RMW_Connext_TypePlugin_create_sample(
  PRESTypePluginEndpointData endpointData)
{
  return PRESTypePluginDefaultEndpointData_createSample(endpointData);
}

void
RMW_Connext_TypePlugin_destroy_sample(
  PRESTypePluginEndpointData endpointData,
  void * sample)
{
  return PRESTypePluginDefaultEndpointData_deleteSample(endpointData, sample);
}

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */
static
RTIBool
RMW_Connext_TypePlugin_create_data(void ** sample, void * user_data)
{
  RMW_Connext_MessageTypeSupport * const type_support =
    reinterpret_cast<RMW_Connext_MessageTypeSupport *>(user_data);

  rcutils_uint8_array_t * data_buffer =
    new (std::nothrow) rcutils_uint8_array_t();
  if (nullptr == data_buffer) {
    return RTI_FALSE;
  }

  const rcutils_allocator_t allocator = rcutils_get_default_allocator();
  size_t buffer_size = 0;

  if (type_support->unbounded()) {
    buffer_size = 0;
  } else {
    buffer_size = type_support->type_serialized_size_max();
  }

  if (RCUTILS_RET_OK !=
    rcutils_uint8_array_init(data_buffer, buffer_size, &allocator))
  {
    delete data_buffer;
    return RTI_FALSE;
  }

  *sample = data_buffer;

  return RTI_TRUE;
}

static
void
RMW_Connext_TypePlugin_destroy_data(void ** sample, void * user_data)
{
  RMW_Connext_MessageTypeSupport * const type_support =
    reinterpret_cast<RMW_Connext_MessageTypeSupport *>(user_data);

  UNUSED_ARG(type_support);

  rcutils_uint8_array_t * data_buffer =
    reinterpret_cast<rcutils_uint8_array_t *>(*sample);

  if (RCUTILS_RET_OK != rcutils_uint8_array_fini(data_buffer)) {
    RMW_CONNEXT_LOG_ERROR("failed to finalize array")
  }

  delete data_buffer;
}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

static
PRESTypePluginParticipantData
RMW_Connext_TypePlugin_on_participant_attached(
  void * registration_data,
  const struct PRESTypePluginParticipantInfo * participant_info,
  RTIBool top_level_registration,
  void * container_plugin_context,
  RTICdrTypeCode * type_code)
{
  UNUSED_ARG(registration_data);
  UNUSED_ARG(top_level_registration);
  UNUSED_ARG(container_plugin_context);

  RMW_Connext_NddsTypeCode * const tc =
    reinterpret_cast<struct RMW_Connext_NddsTypeCode *>(type_code);

  struct PRESTypePluginDefaultParticipantData * const pres_data =
    reinterpret_cast<PRESTypePluginDefaultParticipantData *>(
    PRESTypePluginDefaultParticipantData_new(participant_info));
  if (nullptr == pres_data) {
    return nullptr;
  }

  RMW_Connext_NddsParticipantData * const pdata =
    new (std::nothrow) RMW_Connext_NddsParticipantData(
    pres_data, tc->type_plugin);

  if (nullptr == pdata) {
    PRESTypePluginDefaultParticipantData_delete(pres_data);
    return nullptr;
  }

  return pdata;
}

static
void
RMW_Connext_TypePlugin_on_participant_detached(
  PRESTypePluginParticipantData participant_data)
{
  RMW_Connext_NddsParticipantData * const pdata =
    reinterpret_cast<RMW_Connext_NddsParticipantData *>(participant_data);

  delete pdata;
}

static
PRESTypePluginEndpointData
RMW_Connext_TypePlugin_on_endpoint_attached(
  PRESTypePluginParticipantData participant_data,
  const struct PRESTypePluginEndpointInfo * endpoint_info,
  RTIBool top_level_registration,
  void * containerPluginContext)
{
  UNUSED_ARG(top_level_registration);
  UNUSED_ARG(containerPluginContext);

  RMW_Connext_NddsParticipantData * const pdata =
    reinterpret_cast<RMW_Connext_NddsParticipantData *>(participant_data);

  PRESTypePluginDefaultEndpointData * const epd =
    reinterpret_cast<PRESTypePluginDefaultEndpointData *>(
    PRESTypePluginDefaultEndpointData_newWithNotification(
      &pdata->pres_data,
      endpoint_info,
      (REDAFastBufferPoolBufferInitializeFunction)
      RMW_Connext_TypePlugin_create_data,
      pdata->type_plugin->wrapper,
      (REDAFastBufferPoolBufferFinalizeFunction)
      RMW_Connext_TypePlugin_destroy_data,
      pdata->type_plugin->wrapper,
      NULL, NULL,
      NULL, NULL));

  if (epd == NULL) {
    return NULL;
  }

  epd->userData = pdata->type_plugin->wrapper;

  if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
    const size_t serialized_sample_max_size =
      RMW_Connext_TypePlugin_get_serialized_sample_max_size(
      epd, RTI_FALSE, RTI_CDR_ENCAPSULATION_ID_CDR_BE, 0);

    PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(
      epd, serialized_sample_max_size);

    if (PRESTypePluginDefaultEndpointData_createWriterPool(
        epd,
        endpoint_info,
        (PRESTypePluginGetSerializedSampleMaxSizeFunction)
        RMW_Connext_TypePlugin_get_serialized_sample_max_size,
        epd,
        (PRESTypePluginGetSerializedSampleSizeFunction)
        RMW_Connext_TypePlugin_get_serialized_sample_size,
        epd) == RTI_FALSE)
    {
      PRESTypePluginDefaultEndpointData_delete(epd);
      return NULL;
    }
  }

  return epd;
}

static
void
RMW_Connext_TypePlugin_on_endpoint_detached(
  PRESTypePluginEndpointData endpoint_data)
{
  PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

static
void
RMW_Connext_TypePlugin_return_sample(
  PRESTypePluginEndpointData endpoint_data,
  void * sample,
  void * handle)
{
  /* TODO(asorbini): finalize optional members ? */

  PRESTypePluginDefaultEndpointData_returnSample(
    endpoint_data, sample, handle);
}

static
RTIBool
RMW_Connext_TypePlugin_copy_sample(
  PRESTypePluginEndpointData endpoint_data,
  void * dst,
  const void * src)
{
  UNUSED_ARG(endpoint_data);

  const rcutils_uint8_array_t * src_buffer =
    reinterpret_cast<const rcutils_uint8_array_t *>(src);
  rcutils_uint8_array_t * dst_buffer = reinterpret_cast<rcutils_uint8_array_t *>(dst);

  if (RCUTILS_RET_OK != rcutils_uint8_array_copy(dst_buffer, src_buffer)) {
    return RTI_FALSE;
  }

  return RTI_TRUE;
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */

static
RTIBool
RMW_Connext_TypePlugin_serialize(
  PRESTypePluginEndpointData endpoint_data,
  const void * sample,
  struct RTICdrStream * stream,
  RTIBool serialize_encapsulation,
  RTIEncapsulationId encapsulation_id,
  RTIBool serialize_sample,
  void * endpoint_plugin_qos)
{
  UNUSED_ARG(endpoint_plugin_qos);
  UNUSED_ARG(encapsulation_id);

  PRESTypePluginDefaultEndpointData * const epd =
    reinterpret_cast<PRESTypePluginDefaultEndpointData *>(endpoint_data);
  RMW_Connext_MessageTypeSupport * const type_support =
    reinterpret_cast<RMW_Connext_MessageTypeSupport *>(epd->userData);


  if (!serialize_encapsulation) {
    // currently not supported
    return RTI_FALSE;
  }

  if (!serialize_sample) {
    // currently not supported
    return RTI_FALSE;
  }

  const RMW_Connext_Message * const msg =
    reinterpret_cast<const RMW_Connext_Message *>(sample);

  rcutils_uint8_array_t data_buffer;
  data_buffer.allocator = rcutils_get_default_allocator();
  data_buffer.buffer =
    reinterpret_cast<uint8_t *>(RTICdrStream_getCurrentPosition(stream));
  data_buffer.buffer_length = RTICdrStream_getRemainder(stream);
  data_buffer.buffer_capacity = data_buffer.buffer_length;

  if (!msg->serialized) {
    rmw_ret_t rc = type_support->serialize(msg->user_data, &data_buffer);
    if (RMW_RET_OK != rc) {
      return RTI_FALSE;
    }
    RMW_CONNEXT_LOG_DEBUG_A(
      "serialized: msg=%p, size=%lu",
      msg->user_data, data_buffer.buffer_length)
  } else {
    const rcutils_uint8_array_t * const user_buffer =
      reinterpret_cast<const rcutils_uint8_array_t *>(msg->user_data);
    if (RCUTILS_RET_OK !=
      rcutils_uint8_array_copy(&data_buffer, user_buffer))
    {
      return RTI_FALSE;
    }
  }

  RTICdrStream_setCurrentPosition(
    stream,
    reinterpret_cast<char *>(data_buffer.buffer) + data_buffer.buffer_length);

  return RTI_TRUE;
}

static
RTIBool
RMW_Connext_TypePlugin_deserialize(
  PRESTypePluginEndpointData endpoint_data,
  void ** sample,
  RTIBool * drop_sample,
  struct RTICdrStream * stream,
  RTIBool deserialize_encapsulation,
  RTIBool deserialize_sample,
  void * endpoint_plugin_qos)
{
  UNUSED_ARG(drop_sample);
  UNUSED_ARG(endpoint_plugin_qos);

  if (!deserialize_encapsulation) {
    // Currently not supported
    return RTI_FALSE;
  }
  if (!deserialize_sample) {
    // Currently not supported
    return RTI_FALSE;
  }

  PRESTypePluginDefaultEndpointData * const epd =
    reinterpret_cast<PRESTypePluginDefaultEndpointData *>(endpoint_data);
  RMW_Connext_MessageTypeSupport * const type_support =
    reinterpret_cast<RMW_Connext_MessageTypeSupport *>(epd->userData);
  UNUSED_ARG(type_support);

  rcutils_uint8_array_t * const data_buffer =
    reinterpret_cast<rcutils_uint8_array_t *>(*sample);
  const size_t deserialize_size = RTICdrStream_getRemainder(stream);
  void * const src_ptr = RTICdrStream_getCurrentPosition(stream);

  if (data_buffer->buffer_capacity < deserialize_size) {
    if (RCUTILS_RET_OK !=
      rcutils_uint8_array_resize(data_buffer, deserialize_size))
    {
      return RTI_FALSE;
    }
  }

  memcpy(data_buffer->buffer, src_ptr, deserialize_size);

  data_buffer->buffer_length = deserialize_size;

  RTICdrStream_setCurrentPosition(stream, reinterpret_cast<char *>(src_ptr) + deserialize_size);

  return RTI_TRUE;
}

static
unsigned int
RMW_Connext_TypePlugin_get_serialized_sample_max_size(
  PRESTypePluginEndpointData endpoint_data,
  RTIBool include_encapsulation,
  RTIEncapsulationId encapsulation_id,
  unsigned int current_alignment)
{
  unsigned int initial_alignment = current_alignment;

  UNUSED_ARG(encapsulation_id);

  PRESTypePluginDefaultEndpointData * const epd =
    reinterpret_cast<PRESTypePluginDefaultEndpointData *>(endpoint_data);
  RMW_Connext_MessageTypeSupport * const type_support =
    reinterpret_cast<RMW_Connext_MessageTypeSupport *>(epd->userData);

  // For unbounded types this function will only return the maximum size of
  // the "bounded" part of the message.
  current_alignment += type_support->type_serialized_size_max();

  if (!include_encapsulation) {
    current_alignment -=
      RMW_Connext_MessageTypeSupport::ENCAPSULATION_HEADER_SIZE;
  }

  return current_alignment - initial_alignment;
}

static
unsigned int
RMW_Connext_TypePlugin_get_serialized_sample_min_size(
  PRESTypePluginEndpointData endpoint_data,
  RTIBool include_encapsulation,
  RTIEncapsulationId encapsulation_id,
  unsigned int current_alignment)
{
  return RMW_Connext_TypePlugin_get_serialized_sample_max_size(
    endpoint_data,
    include_encapsulation,
    encapsulation_id,
    current_alignment);
}

static
unsigned int
RMW_Connext_TypePlugin_get_serialized_sample_size(
  PRESTypePluginEndpointData endpoint_data,
  RTIBool include_encapsulation,
  RTIEncapsulationId encapsulation_id,
  unsigned int current_alignment,
  const void * sample)
{
  UNUSED_ARG(encapsulation_id);

  unsigned int initial_alignment = current_alignment;

  if (sample == NULL) {
    return 0;
  }
  if (endpoint_data == NULL) {
    return 0;
  }

  PRESTypePluginDefaultEndpointData * const epd =
    reinterpret_cast<PRESTypePluginDefaultEndpointData *>(endpoint_data);
  RMW_Connext_MessageTypeSupport * const type_support =
    reinterpret_cast<RMW_Connext_MessageTypeSupport *>(epd->userData);

  const RMW_Connext_Message * const msg =
    reinterpret_cast<const RMW_Connext_Message *>(sample);

  if (msg->serialized) {
    const rcutils_uint8_array_t * const serialized_msg =
      reinterpret_cast<const rcutils_uint8_array_t *>(msg->user_data);
    current_alignment += serialized_msg->buffer_length;
  } else {
    current_alignment +=
      type_support->serialized_size_max(
      msg->user_data, include_encapsulation);
  }

  if (!include_encapsulation) {
    current_alignment -=
      RMW_Connext_MessageTypeSupport::ENCAPSULATION_HEADER_SIZE;
  }

  return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */
static
PRESTypePluginKeyKind
RMW_Connext_TypePlugin_get_key_kind(void)
{
  return PRES_TYPEPLUGIN_NO_KEY;
}

/* ------------------------------------------------------------------------
* Plug-in Life-cycle
* ------------------------------------------------------------------------ */
static
void
RMW_Connext_TypePlugin_initialize(
  struct PRESTypePlugin * const plugin,
  struct RTICdrTypeCode * const type_code,
  const char * const type_name)
{
  const struct PRESTypePluginVersion PLUGIN_VERSION =
    PRES_TYPE_PLUGIN_VERSION_2_0;

  plugin->version = PLUGIN_VERSION;
  plugin->typeCode = type_code;
  plugin->languageKind = PRES_TYPEPLUGIN_NON_DDS_TYPE;
  plugin->endpointTypeName = type_name;
  plugin->typeCodeName = type_name;

  /* Partcipant/endpoint events */
  plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    RMW_Connext_TypePlugin_on_participant_attached;
  plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    RMW_Connext_TypePlugin_on_participant_detached;
  plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    RMW_Connext_TypePlugin_on_endpoint_attached;
  plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    RMW_Connext_TypePlugin_on_endpoint_detached;

  /* Sample management */
  plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    RMW_Connext_TypePlugin_copy_sample;
  plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    RMW_Connext_TypePlugin_create_sample;
  plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    RMW_Connext_TypePlugin_destroy_sample;
  plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    RMW_Connext_TypePlugin_get_sample;
  plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    RMW_Connext_TypePlugin_return_sample;
  plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    NULL /* TODO(asorbini): implement? */;

  /* Serialization/deserialization */
  plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    RMW_Connext_TypePlugin_serialize;
  plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    RMW_Connext_TypePlugin_deserialize;
  plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    RMW_Connext_TypePlugin_get_serialized_sample_max_size;
  plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    RMW_Connext_TypePlugin_get_serialized_sample_min_size;
  plugin->getBuffer =
    (PRESTypePluginGetBufferFunction)
    RMW_Connext_TypePlugin_get_buffer;
  plugin->returnBuffer =
    (PRESTypePluginReturnBufferFunction)
    RMW_Connext_TypePlugin_return_buffer;
  plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    RMW_Connext_TypePlugin_get_serialized_sample_size;

  /* Key management */
  plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    RMW_Connext_TypePlugin_get_key_kind;
  plugin->serializeKeyFnc = NULL;
  plugin->deserializeKeyFnc = NULL;
  plugin->getKeyFnc = NULL;
  plugin->returnKeyFnc = NULL;
  plugin->instanceToKeyFnc = NULL;
  plugin->keyToInstanceFnc = NULL;
  plugin->getSerializedKeyMaxSizeFnc = NULL;
  plugin->instanceToKeyHashFnc = NULL;
  plugin->serializedSampleToKeyHashFnc = NULL;
  plugin->serializedKeyToKeyHashFnc = NULL;

#if !RMW_CONNEXT_DDS_API_PRO_LEGACY
  /* This functions are not part of the pre-6.x type plugin */
  plugin->isMetpType = RTI_FALSE;
  plugin->getWriterLoanedSampleFnc = NULL;
  plugin->returnWriterLoanedSampleFnc = NULL;
  plugin->returnWriterLoanedSampleFromCookieFnc = NULL;
  plugin->validateWriterLoanedSampleFnc = NULL;
  plugin->setWriterLoanedSampleSerializedStateFnc = NULL;
  plugin->getBufferWithParams = NULL;
  plugin->returnBufferWithParams = NULL;
#endif /* RMW_CONNEXT_DDS_API_PRO_LEGACY */
}

/******************************************************************************
 * Type registration API
 ******************************************************************************/

RMW_Connext_MessageTypeSupport *
rmw_connextdds_register_type_support(
  rmw_context_impl_t * const ctx,
  const rosidl_message_type_support_t * const type_supports,
  DDS_DomainParticipant * const participant,
  bool & registered,
  const RMW_Connext_MessageType message_type,
  const void * const intro_members,
  const bool intro_members_cpp,
  const char * const type_name)
{
  UNUSED_ARG(ctx);

  registered = false;

  RMW_Connext_MessageTypeSupport * type_support_res = nullptr;

  RMW_Connext_MessageTypeSupport * type_support = nullptr;
  try {
    type_support = new RMW_Connext_MessageTypeSupport(
      message_type, type_supports, type_name);
  } catch (const std::exception & e) {
    RMW_CONNEXT_LOG_ERROR_A("failed to create type support: %s", e.what())
  }

  if (nullptr == type_support) {
    return nullptr;
  }

  auto scope_exit_support_delete =
    rcpputils::make_scope_exit(
    [type_support]()
    {
      delete type_support;
    });

  /* We can't use DDS_DomainParticipant_get_type_pluginI because the
     type plugin gets copied into the database record, so lookup the
     associated type code and retrieve the custom type plugin from it */

  const RMW_Connext_NddsTypeCode * const tc =
    (const RMW_Connext_NddsTypeCode *)
    DDS_DomainParticipant_get_typecode(
    participant, type_support->type_name());

  if (nullptr == tc) {
    struct REDAFastBufferPoolProperty
      pool_prop = REDA_FAST_BUFFER_POOL_PROPERTY_DEFAULT;

    struct REDAFastBufferPool * const pool_samples =
      REDAFastBufferPool_new(
      sizeof(rcutils_uint8_array_t),
      RTIOsapiAlignment_getDefaultAlignment(),
      &pool_prop);

    if (nullptr == pool_samples) {
      return nullptr;
    }
    auto scope_exit_pool_samples_delete =
      rcpputils::make_scope_exit(
      [pool_samples]()
      {
        REDAFastBufferPool_delete(pool_samples);
      });

    DDS_TypeCode * dds_tc = nullptr;
    struct RMW_Connext_TypeCodePtrSeq tc_cache = DDS_SEQUENCE_INITIALIZER;

    struct RMW_Connext_TypeCodePtrSeq * const tc_cache_ptr = &tc_cache;
    auto scope_exit_tc_cache_delete =
      rcpputils::make_scope_exit(
      [tc_cache_ptr]()
      {
        rmw_connextdds_release_typecode_cache(tc_cache_ptr);
      });

#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
    dds_tc = rmw_connextdds_create_typecode(
      type_supports,
      type_support->type_name(),
      intro_members,
      intro_members_cpp,
      &tc_cache);
    if (nullptr == dds_tc) {
      RMW_CONNEXT_LOG_ERROR_A(
        "failed to generate DDS type code: %s",
        type_support->type_name())
      return nullptr;
    }
#else
    UNUSED_ARG(intro_members);
    UNUSED_ARG(intro_members_cpp);
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */

    auto scope_exit_dds_tc_delete =
      rcpputils::make_scope_exit(
      [dds_tc]()
      {
        if (nullptr == dds_tc) {
          rmw_connextdds_delete_typecode(dds_tc);
        }
      });

    RMW_Connext_NddsTypePluginI * type_plugin =
      new (std::nothrow) RMW_Connext_NddsTypePluginI(
      type_support,
      pool_samples,
      dds_tc,
      &tc_cache);

    if (nullptr == type_plugin) {
      RMW_CONNEXT_LOG_ERROR_A(
        "failed to allocate type plugin interface: %s",
        type_support->type_name())
      return nullptr;
    }

    scope_exit_tc_cache_delete.cancel();
    scope_exit_dds_tc_delete.cancel();

    scope_exit_pool_samples_delete.cancel();

    auto scope_exit_intf_delete =
      rcpputils::make_scope_exit(
      [type_plugin]()
      {
        delete type_plugin;
      });

    RMW_Connext_TypePlugin_initialize(
      &type_plugin->base,
      &type_plugin->type_code.base,
      type_support->type_name());

    if (DDS_RETCODE_OK !=
      DDS_DomainParticipant_register_type(
        participant,
        type_support->type_name(),
        &type_plugin->base,
        NULL))
    {
      RMW_CONNEXT_LOG_ERROR_A(
        "failed to register type plugin: %s",
        type_support->type_name())
      return nullptr;
    }

    RMW_CONNEXT_LOG_DEBUG_A(
      "registered type: "
      "name=%s, type_support=%p",
      type_support->type_name(), (void *)type_support)

    scope_exit_intf_delete.cancel();

    type_support_res = type_support;
    scope_exit_support_delete.cancel();
  } else {
    tc->type_plugin->attached_count += 1;
    type_support_res = tc->type_plugin->wrapper;
  }

  return type_support_res;
}

rmw_ret_t
rmw_connextdds_unregister_type_support(
  rmw_context_impl_t * const ctx,
  DDS_DomainParticipant * const participant,
  const char * const type_name)
{
  UNUSED_ARG(ctx);

  const RMW_Connext_NddsTypeCode * const tc =
    (const RMW_Connext_NddsTypeCode *)
    DDS_DomainParticipant_get_typecode(participant, type_name);

  if (nullptr == tc) {
    RMW_CONNEXT_LOG_ERROR_A("failed to lookup type: %s", type_name)
    return RMW_RET_ERROR;
  }

  tc->type_plugin->attached_count -= 1;

  if (tc->type_plugin->attached_count == 0) {
    /* Cache type_name into a string, since may be deallocated
       by DDS_DomainParticipant_unregister_type() */
    std::string tname(type_name);

    if (DDS_RETCODE_OK !=
      DDS_DomainParticipant_unregister_type(
        participant, type_name))
    {
      RMW_CONNEXT_LOG_ERROR_A(
        "failed to unregister type: %s", tname.c_str())
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}
