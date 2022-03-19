// Copyright 2021 Real-Time Innovations, Inc. (RTI)
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
#include <new>
#include <string>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rmw_connextdds/custom_sql_filter.hpp"

#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO

extern "C" {
// This is an internal function from RTI Connext DDS which allows a filter to
// be registered as "built-in". We need this because we want this custom filter
// to be a replacement for the built-in SQL-like filter.
DDS_ReturnCode_t
DDS_ContentFilter_register_filter(
  DDS_DomainParticipant * participant,
  const char * name,
  const struct DDS_ContentFilter * filter,
  const DDS_ContentFilterEvaluateFunction evaluateOnSerialized,
  const DDS_ContentFilterWriterEvaluateFunction writerEvaluateOnSerialized,
  const DDS_ContentFilterQueryFunction query,
  DDS_Boolean isBuiltin);
}

struct RTI_CustomSqlFilterProgram
{
  void * base{nullptr};
  std::string expression;
  std::string type_name;
  DDS_StringSeq expression_params DDS_SEQUENCE_INITIALIZER;
  ~RTI_CustomSqlFilterProgram()
  {
    DDS_StringSeq_finalize(&this->expression_params);
  }
};

struct RTI_CustomSqlFilterWriterData
{
  void * base{nullptr};
  size_t filtered_readers_count{0};
  size_t unfiltered_readers_count{0};
  std::vector<DDS_Cookie_t *> unfiltered_readers_buffer;
  std::vector<DDS_Cookie_t *> matched_readers_buffer;
  DDS_CookieSeq matched_readers DDS_SEQUENCE_INITIALIZER;
  REDASkiplistDescription readers_desc;
  REDASkiplist readers;
};

struct RTI_CustomSqlFilterReaderData
{
  DDS_Cookie_t cookie;
  REDAWeakReference wr;
  std::string expression;

  explicit RTI_CustomSqlFilterReaderData(REDAWeakReference * const wr)
  : cookie(DDS_COOKIE_DEFAULT),
    wr(*wr)
  {
    DDS_Cookie_t_initialize(&this->cookie);
    DDS_OctetSeq_loan_contiguous(
      &this->cookie.value,
      reinterpret_cast<DDS_Octet *>(&this->wr),
      sizeof(struct REDAWeakReference),
      sizeof(struct REDAWeakReference));
  }
};

using RTI_CustomSqlFilterData = rti_connext_dds_custom_sql_filter::CustomSqlFilterData;

RTI_CustomSqlFilterData::CustomSqlFilterData()
  : base(DDS_SQLFILTER_QOS_DEFAULT)
{
}

DDS_ReturnCode_t
RTI_CustomSqlFilterData::set_memory_management_property(
  const DDS_DomainParticipantQos & dp_qos)
{
  static const DDS_SqlFilterMemoryManagementQos DEFAULT =
    DDS_SqlFilterMemoryManagementQos_INITIALIZER;
  this->base.memory_management = DEFAULT;

  auto properties = const_cast<DDS_PropertyQosPolicy *>(&dp_qos.property);

  const DDS_Property_t * property = DDS_PropertyQosPolicyHelper_lookup_property(
    properties,
    DDS_CONTENT_FILTER_SQL_DESERIALIZED_SAMPLE_MIN_BUFFER_SIZE_PROPERTY_NAME);

  if (nullptr != property) {
    if (!sscanf(
        property->value, "%d",
        &this->base.memory_management.buffer_min_size))
    {
      // TODO(asorbini) log error
      return DDS_RETCODE_ERROR;
    }
  }

  property = DDS_PropertyQosPolicyHelper_lookup_property(
    properties,
    DDS_CONTENT_FILTER_SQL_DESERIALIZED_SAMPLE_TRIM_TO_SIZE_PROPERTY_NAME);

  if (nullptr != property) {
    if (!REDAString_iCompare(property->value, "1") ||
      !REDAString_iCompare(property->value, "true") ||
      !REDAString_iCompare(property->value, "yes"))
    {
      this->base.memory_management.trim_buffer = DDS_BOOLEAN_TRUE;
    }
  }

  return DDS_RETCODE_OK;
}

static
int
RTI_CustomSqlFilter_compare_reader_data(
  const void * left,
  const void * right)
{
  RTI_CustomSqlFilterReaderData * const l =
    const_cast<RTI_CustomSqlFilterReaderData *>(
    static_cast<const RTI_CustomSqlFilterReaderData *>(left));
  RTI_CustomSqlFilterReaderData * const r =
    const_cast<RTI_CustomSqlFilterReaderData *>(
    static_cast<const RTI_CustomSqlFilterReaderData *>(right));

  return RTIOsapiMemory_compare(
    DDS_OctetSeq_get_contiguous_buffer(&l->cookie.value),
    DDS_OctetSeq_get_contiguous_buffer(&r->cookie.value),
    sizeof(struct REDAWeakReference));
}

static
DDS_ReturnCode_t
RTI_CustomSqlFilter_compile(
  void * filter_data,
  void ** newHandle,
  const char * expression,
  const struct DDS_StringSeq * param_seq,
  const struct DDS_TypeCode * typeCode,
  const char * typeClassName,
  void * oldhandle)
{
  *newHandle = nullptr;

  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);

  RTI_CustomSqlFilterProgram * program =
    new (std::nothrow) RTI_CustomSqlFilterProgram();
  if (nullptr == program) {
    // TODO(asorbini) log error
    return DDS_RETCODE_ERROR;
  }

  auto scope_exit_program = rcpputils::make_scope_exit(
    [program]()
    {
      delete program;
    });

  RTI_CustomSqlFilterProgram * old_program =
    static_cast<RTI_CustomSqlFilterProgram *>(oldhandle);
  void * const old_program_base =
    (nullptr != old_program) ? old_program->base : nullptr;

  program->type_name = typeClassName;
  program->expression = expression;
  if (!DDS_StringSeq_copy(&program->expression_params, param_seq)) {
    // TODO(asorbini) log error
    return DDS_RETCODE_ERROR;
  }

  if (!program->expression.empty()) {
    const DDS_ReturnCode_t retcode =
      DDS_SqlFilter_compile(
      &cft_data->base,
      &program->base,
      expression,
      param_seq,
      typeCode,
      typeClassName,
      old_program_base);
    if (DDS_RETCODE_OK != retcode) {
      // TODO(asorbini) log error
      return retcode;
    }
  }

  // Delete old_program if we didn't actually compile anything.
  // In the other case, Connext doesn't delete the old program to
  // avoid a possible race condition, so we do the same here and
  // leave it alone. TODO(asorbini) This is likely a memory leak.
  if (nullptr != old_program && nullptr == old_program->base) {
    delete old_program;
  }

  *newHandle = program;

  scope_exit_program.cancel();
  return DDS_RETCODE_OK;
}

static
DDS_ReturnCode_t
RTI_CustomSqlFilter_writer_attach(
  void * filter_data,
  void ** writer_filter_data,
  void * reserved)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterWriterData * writer_data =
    new (std::nothrow) RTI_CustomSqlFilterWriterData();
  if (nullptr == writer_data) {
    // TODO(asorbini) log error
    return DDS_RETCODE_ERROR;
  }
  auto scope_exit_data = rcpputils::make_scope_exit(
    [writer_data]()
    {
      delete writer_data;
    });

  if (!REDASkiplist_newDefaultAllocator(
      &writer_data->readers_desc,
      REDASkiplist_getOptimumMaximumLevel(REDA_FAST_BUFFER_POOL_UNLIMITED),
      1))
  {
    // TODO(asorbini) log error
    return DDS_RETCODE_ERROR;
  }

  REDASkiplist_init(
    &writer_data->readers,
    &writer_data->readers_desc,
    RTI_CustomSqlFilter_compare_reader_data,
    nullptr,
    0,
    0);

  DDS_ReturnCode_t rc =
    DDS_SqlFilter_writerAttach(&cft_data->base, &writer_data->base, reserved);
  if (DDS_RETCODE_OK != rc) {
    // TODO(asorbini) log error
    return rc;
  }
  *writer_filter_data = writer_data;
  scope_exit_data.cancel();
  return DDS_RETCODE_OK;
}

void
RTI_CustomSqlFilter_writer_detach(
  void * filter_data,
  void * writer_filter_data)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterWriterData * const writer_data =
    static_cast<RTI_CustomSqlFilterWriterData *>(writer_filter_data);

  DDS_SqlFilter_writerDetach(&cft_data->base, writer_data->base);

  REDASkiplistNode * node = nullptr;
  REDASkiplist_gotoTopNode(&writer_data->readers, &node);
  while (REDASkiplist_gotoNextNode(&writer_data->readers, &node)) {
    RTI_CustomSqlFilterReaderData * const rdata =
      static_cast<RTI_CustomSqlFilterReaderData *>(REDASkiplistNode_getUserData(node));
    delete rdata;
  }

  REDASkiplist_deleteDefaultAllocator(&writer_data->readers_desc);
  delete writer_data;
}

static
void
RTI_CustomSqlFilter_update_unfiltered_list(
  RTI_CustomSqlFilterWriterData * const writer_data)
{
  writer_data->unfiltered_readers_buffer.clear();
  writer_data->unfiltered_readers_buffer.reserve(writer_data->unfiltered_readers_count);

  REDASkiplistNode * node = nullptr;
  REDASkiplist_gotoTopNode(&writer_data->readers, &node);
  while (REDASkiplist_gotoNextNode(&writer_data->readers, &node)) {
    RTI_CustomSqlFilterReaderData * const rdata =
      static_cast<RTI_CustomSqlFilterReaderData *>(REDASkiplistNode_getUserData(node));
    if (rdata->expression.empty()) {
      writer_data->unfiltered_readers_buffer.push_back(&rdata->cookie);
    }
  }
}

static
DDS_ReturnCode_t
RTI_CustomSqlFilter_writer_compile(
  void * filter_data,
  void * writer_filter_data,
  struct DDS_ExpressionProperty * prop,
  const char * expression,
  const struct DDS_StringSeq * parameters,
  const struct DDS_TypeCode * type_code,
  const char * type_class_name,
  const struct DDS_Cookie_t * cookie)
{
  const bool empty_expr = expression[0] == '\0';

  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterWriterData * const writer_data =
    static_cast<RTI_CustomSqlFilterWriterData *>(writer_filter_data);

  bool regen_unfiltered = false;
  bool new_reader = false;

  RTI_CustomSqlFilterReaderData * reader_data = nullptr;
  bool reader_data_added = false;
  bool reader_data_compiled = false;
  auto scope_exit_reader_data = rcpputils::make_scope_exit(
    [cft_data, writer_data, cookie,
    &reader_data_added, &reader_data_compiled, &reader_data]()
    {
      if (reader_data_compiled) {
        DDS_SqlFilter_writerFinalize(
          &cft_data->base, writer_data->base, cookie);
      }
      if (reader_data_added) {
        REDASkiplist_removeNodeEA(&writer_data->readers, reader_data);
      }
      if (nullptr != reader_data) {
        delete reader_data;
      }
    });
  REDAWeakReference cookie_wr;
  RTIOsapiMemory_copy(
    &cookie_wr,
    DDS_OctetSeq_get_contiguous_buffer(&cookie->value),
    sizeof(struct REDAWeakReference));
  RTI_CustomSqlFilterReaderData lookup_data(&cookie_wr);

  REDASkiplistNode * node = nullptr;
  RTIBool precise_match = RTI_FALSE;
  REDASkiplist_findNode(
    &writer_data->readers,
    const_cast<const REDASkiplistNode **>(&node),
    &precise_match,
    static_cast<void *>(&lookup_data));
  if (!precise_match) {
    node = nullptr;
  }
  if (nullptr != node) {
    reader_data =
      static_cast<RTI_CustomSqlFilterReaderData *>(REDASkiplistNode_getUserData(node));
    reader_data_added = true;

    if (!reader_data->expression.empty() && empty_expr) {
      DDS_SqlFilter_writerFinalize(
        &cft_data->base, writer_data->base, cookie);
      writer_data->unfiltered_readers_count += 1;
      writer_data->filtered_readers_count -= 1;
    } else if (reader_data->expression.empty() && !empty_expr) {
      // Regenerate the list of unfiltered readers
      writer_data->unfiltered_readers_count -= 1;
      writer_data->filtered_readers_count += 1;
      regen_unfiltered = true;
    }

  } else {
    new_reader = true;
    reader_data = new (std::nothrow) RTI_CustomSqlFilterReaderData(&cookie_wr);
    if (nullptr == reader_data) {
      // TODO(asorbini) log error
      return DDS_RETCODE_ERROR;
    }

    if (REDASkiplist_assertNodeEA(
        &writer_data->readers,
        nullptr,
        reader_data,
        0,
        0) == nullptr)
    {
      // TODO(asorbini) log error
      return DDS_RETCODE_ERROR;
    }
    reader_data_added = true;

    if (empty_expr) {
      writer_data->unfiltered_readers_count += 1;
    } else {
      writer_data->filtered_readers_count += 1;
    }
  }

  reader_data->expression = expression;

  if (empty_expr) {
    prop->key_only_filter = DDS_BOOLEAN_FALSE;
    prop->writer_side_filter_optimization = DDS_BOOLEAN_FALSE;
  } else {
    DDS_ReturnCode_t rc = DDS_SqlFilter_writerCompile(
      &cft_data->base,
      writer_data->base,
      prop,
      expression,
      parameters,
      type_code,
      type_class_name,
      cookie);
    if (DDS_RETCODE_OK != rc) {
      // TODO(asorbini) log error
      return rc;
    }
    reader_data_compiled = true;
  }

  const size_t readers_size = writer_data->matched_readers_buffer.size();
  if (new_reader) {
    writer_data->matched_readers_buffer.reserve(readers_size + 1);
  }

  if (regen_unfiltered) {
    RTI_CustomSqlFilter_update_unfiltered_list(writer_data);
  } else if (new_reader && empty_expr) {
    writer_data->unfiltered_readers_buffer.push_back(&reader_data->cookie);
  }

  scope_exit_reader_data.cancel();
  return DDS_RETCODE_OK;
}

DDS_CookieSeq *
RTI_CustomSqlFilter_writer_evaluated_result(
  RTI_CustomSqlFilterWriterData * const writer_data,
  DDS_CookieSeq * const sql_matched)
{
  if (DDS_CookieSeq_get_maximum(&writer_data->matched_readers) > 0) {
    DDS_CookieSeq_unloan(&writer_data->matched_readers);
  }

  const size_t sql_matched_len =
    (nullptr != sql_matched) ? DDS_CookieSeq_get_length(sql_matched) : 0;
  const size_t unfiltered_size = writer_data->unfiltered_readers_buffer.size();
  size_t readers_size = sql_matched_len + unfiltered_size;
  size_t matched_readers_start = 0;

  writer_data->matched_readers_buffer.resize(readers_size);

  if (sql_matched_len > 0) {
    DDS_Cookie_t ** const sql_matched_buffer =
      DDS_CookieSeq_get_discontiguous_buffer(sql_matched);
    memcpy(
      &(writer_data->matched_readers_buffer[matched_readers_start]),
      sql_matched_buffer,
      sizeof(DDS_Cookie_t *) * sql_matched_len);
    matched_readers_start += sql_matched_len;
  }

  if (unfiltered_size > 0) {
    memcpy(
      &(writer_data->matched_readers_buffer[matched_readers_start]),
      &(writer_data->unfiltered_readers_buffer[0]),
      sizeof(DDS_Cookie_t *) * unfiltered_size);
  }

  readers_size = writer_data->matched_readers_buffer.size();
  if (readers_size > 0) {
    if (!DDS_CookieSeq_loan_discontiguous(
        &writer_data->matched_readers,
        &(writer_data->matched_readers_buffer[0]),
        readers_size,
        readers_size))
    {
      // TODO(asorbini) log error
      return nullptr;
    }
  }

  return &writer_data->matched_readers;
}

DDS_CookieSeq *
RTI_CustomSqlFilter_writer_evaluate(
  void * filter_data,
  void * writer_filter_data,
  const void * sample,
  const DDS_FilterSampleInfo * meta_data)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterWriterData * const writer_data =
    static_cast<RTI_CustomSqlFilterWriterData *>(writer_filter_data);

  DDS_CookieSeq * sql_matched = nullptr;

  if (writer_data->filtered_readers_count > 0) {
    sql_matched = DDS_SqlFilter_writerEvaluate(
      &cft_data->base, writer_data->base, sample, meta_data);
    if (nullptr == sql_matched) {
      // TODO(asorbini) log error
      return nullptr;
    }
  }

  return RTI_CustomSqlFilter_writer_evaluated_result(writer_data, sql_matched);
}

void
RTI_CustomSqlFilter_writer_finalize(
  void * filter_data,
  void * writer_filter_data,
  const struct DDS_Cookie_t * cookie)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterWriterData * const writer_data =
    static_cast<RTI_CustomSqlFilterWriterData *>(writer_filter_data);

  REDAWeakReference cookie_wr;
  RTIOsapiMemory_copy(
    &cookie_wr,
    DDS_OctetSeq_get_contiguous_buffer(&cookie->value),
    sizeof(struct REDAWeakReference));
  RTI_CustomSqlFilterReaderData lookup_data(&cookie_wr);

  REDASkiplistNode * node =
    const_cast<REDASkiplistNode *>(
    REDASkiplist_removeNodeEA(&writer_data->readers, &lookup_data));
  if (nullptr != node) {
    RTI_CustomSqlFilterReaderData * reader_data =
      static_cast<RTI_CustomSqlFilterReaderData *>(REDASkiplistNode_getUserData(node));
    if (reader_data->expression.empty()) {
      writer_data->unfiltered_readers_count -= 1;
      RTI_CustomSqlFilter_update_unfiltered_list(writer_data);
    } else {
      writer_data->filtered_readers_count -= 1;
      DDS_SqlFilter_writerFinalize(&cft_data->base, writer_data->base, cookie);
    }
    delete reader_data;
    REDASkiplist_deleteNode(&writer_data->readers, node);
  }
}

void
RTI_CustomSqlFilter_writer_return_loan(
  void * filter_data,
  void * writer_filter_data,
  struct DDS_CookieSeq * cookies)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  (void)cft_data;
  RTI_CustomSqlFilterWriterData * const writer_data =
    static_cast<RTI_CustomSqlFilterWriterData *>(writer_filter_data);
  (void)writer_data;
  DDS_CookieSeq_set_length(cookies, 0);
}

DDS_Boolean
RTI_CustomSqlFilter_evaluate(
  void * filter_data,
  void * handle,
  const void * sample,
  const struct DDS_FilterSampleInfo * meta_data)
{

  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterProgram * const program =
    static_cast<RTI_CustomSqlFilterProgram *>(handle);

  if (nullptr == program->base) {
    return DDS_BOOLEAN_TRUE;
  }

  return DDS_SqlFilter_evaluate(&cft_data->base, program->base, sample, meta_data);
}

void
RTI_CustomSqlFilter_finalize(void * filter_data, void * handle)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterProgram * const program =
    static_cast<RTI_CustomSqlFilterProgram *>(handle);

  if (nullptr != program->base) {
    DDS_SqlFilter_finalize(&cft_data->base, program->base);
  }

  delete program;
}

DDS_Boolean
RTI_CustomSqlFilter_evaluate_on_serialized(
  void * filter_data,
  void * handle,
  const void * sample,
  const struct DDS_FilterSampleInfo * meta_data)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterProgram * const program =
    static_cast<RTI_CustomSqlFilterProgram *>(handle);
  DDS_Boolean accepted = DDS_BOOLEAN_TRUE;
  if (nullptr != program->base) {
    accepted = DDS_SqlFilter_evaluateOnSerialized(
      &cft_data->base, program->base, sample, meta_data);
  }
  return accepted;
}

struct DDS_CookieSeq *
RTI_CustomSqlFilter_writer_evaluate_on_serialized(
  void * filter_data,
  void * writer_filter_data,
  const void * sample,
  const struct DDS_FilterSampleInfo * meta_data)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  RTI_CustomSqlFilterWriterData * const writer_data =
    static_cast<RTI_CustomSqlFilterWriterData *>(writer_filter_data);

  DDS_CookieSeq * sql_matched = nullptr;

  if (writer_data->filtered_readers_count > 0) {
    sql_matched = DDS_SqlFilter_writerEvaluateOnSerialized(
      &cft_data->base, writer_data->base, sample, meta_data);
    if (nullptr == sql_matched) {
      // TODO(asorbini) log error
      return nullptr;
    }
  }

  return RTI_CustomSqlFilter_writer_evaluated_result(writer_data, sql_matched);
}

DDS_Long
RTI_CustomSqlFilter_query(void * filter_data, void * handle)
{
  RTI_CustomSqlFilterData * const cft_data =
    static_cast<RTI_CustomSqlFilterData *>(filter_data);
  (void)cft_data;

  RTI_CustomSqlFilterProgram * const program =
    static_cast<RTI_CustomSqlFilterProgram *>(handle);

  if (nullptr == program->base) {
    return 0;
  }

  return DDS_SqlFilter_query(filter_data, program->base);
}

DDS_ReturnCode_t
rti_connext_dds_custom_sql_filter::register_content_filter(
  DDS_DomainParticipant * const participant,
  rti_connext_dds_custom_sql_filter::CustomSqlFilterData * const filter_data)
{
  DDS_DomainParticipantQos dp_qos = DDS_DomainParticipantQos_INITIALIZER;
  auto scope_exit_qos = rcpputils::make_scope_exit(
    [&dp_qos]()
    {
      if (DDS_RETCODE_OK != DDS_DomainParticipantQos_finalize(&dp_qos)) {
        // TODO(asorbini) log error
      }
    });


  if (DDS_RETCODE_OK != DDS_DomainParticipant_get_qos(participant, &dp_qos)) {
    // TODO(asorbini) log error
    return DDS_RETCODE_ERROR;
  }

  DDS_ReturnCode_t rc = filter_data->set_memory_management_property(dp_qos);
  if (DDS_RETCODE_OK != rc) {
    // TODO(asorbini) log error
    return rc;
  }

  DDS_ContentFilter filter = DDS_ContentFilter_INITIALIZER;
  filter.compile = RTI_CustomSqlFilter_compile;
  filter.writer_attach = RTI_CustomSqlFilter_writer_attach;
  filter.writer_compile = RTI_CustomSqlFilter_writer_compile;
  filter.writer_detach = RTI_CustomSqlFilter_writer_detach;
  filter.writer_evaluate = RTI_CustomSqlFilter_writer_evaluate;
  filter.writer_finalize = RTI_CustomSqlFilter_writer_finalize;
  filter.writer_return_loan = RTI_CustomSqlFilter_writer_return_loan;
  filter.evaluate = RTI_CustomSqlFilter_evaluate;
  filter.finalize = RTI_CustomSqlFilter_finalize;
  filter.filter_data = filter_data;

  rc = DDS_ContentFilter_register_filter(
    participant,
    PLUGIN_NAME,
    &filter,
    RTI_CustomSqlFilter_evaluate_on_serialized,
    RTI_CustomSqlFilter_writer_evaluate_on_serialized,
    RTI_CustomSqlFilter_query,
    DDS_BOOLEAN_TRUE);
  if (DDS_RETCODE_OK != rc) {
    // TODO(asorbini) log error
    return DDS_RETCODE_ERROR;
  }

  return DDS_RETCODE_OK;
}

const char * const
rti_connext_dds_custom_sql_filter::PLUGIN_NAME = "RTI_CONNEXTDDS_CUSTOM_SQL_FILTER";

#endif  // RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO
