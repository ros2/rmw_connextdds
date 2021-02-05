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

#ifndef RMW_CONNEXTDDS__RMW_IMPL_HPP_
#define RMW_CONNEXTDDS__RMW_IMPL_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "rmw_connextdds/context.hpp"
#include "rmw_connextdds/type_support.hpp"
#include "rmw_connextdds/demangle.hpp"
#include "rmw_connextdds/namespace_prefix.hpp"
#include "rmw_connextdds/rmw_api_impl.hpp"

#include "rcutils/types/uint8_array.h"
#include "rcpputils/thread_safety_annotations.hpp"

#if !RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS

/******************************************************************************
 * Symbols provided by rmw/incompatible_qos.h from Foxy onward
 ******************************************************************************/

typedef enum RMW_PUBLIC_TYPE rmw_qos_policy_kind_t
{
  RMW_QOS_POLICY_INVALID = 1 << 0,
  RMW_QOS_POLICY_DURABILITY = 1 << 1,
  RMW_QOS_POLICY_DEADLINE = 1 << 2,
  RMW_QOS_POLICY_LIVELINESS = 1 << 3,
  RMW_QOS_POLICY_RELIABILITY = 1 << 4,
  RMW_QOS_POLICY_HISTORY = 1 << 5,
  RMW_QOS_POLICY_LIFESPAN = 1 << 6
} rmw_qos_policy_kind_t;

/******************************************************************************
 * rmw_qos_incompatible_event_status_t is defined by rmw/incompatible_qos.h
 * from Foxy onward.
 ******************************************************************************/
struct RMW_PUBLIC_TYPE rmw_qos_incompatible_event_status_t
{
  int32_t total_count;
  int32_t total_count_change;
  rmw_qos_policy_kind_t last_policy_kind;
};

typedef struct rmw_qos_incompatible_event_status_t rmw_requested_qos_incompatible_event_status_t;

typedef struct rmw_qos_incompatible_event_status_t rmw_offered_qos_incompatible_event_status_t;

#endif /* !RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS */

#if !RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT

/******************************************************************************
 * Define value for RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, and
 * RMW_EVENT_OFFERED_QOS_INCOMPATIBLE as invalid values,
 * since they're not defined by rmw_event_type_t.
 ******************************************************************************/
#define RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE    (RMW_EVENT_INVALID + 1)
#define RMW_EVENT_OFFERED_QOS_INCOMPATIBLE      (RMW_EVENT_INVALID + 2)

#endif /* !RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT */

#if !RMW_CONNEXT_HAVE_MESSAGE_LOST
/******************************************************************************
 * Define value for RMW_EVENT_MESSAGE_LOST as an invalid value,
 * since it's not defined by rmw_event_type_t.
 ******************************************************************************/
#define RMW_EVENT_MESSAGE_LOST  (RMW_EVENT_INVALID + 3)

typedef struct RMW_PUBLIC_TYPE rmw_message_lost_status_t
{
  size_t total_count;
  size_t total_count_change;
} rmw_message_lost_status_t;
#endif /* !RMW_CONNEXT_HAVE_MESSAGE_LOST */

#if !RMW_CONNEXT_HAVE_SERVICE_INFO
typedef rcutils_time_point_value_t rmw_time_point_value_t;

typedef struct RMW_PUBLIC_TYPE rmw_service_info_t
{
  rmw_time_point_value_t source_timestamp;
  rmw_time_point_value_t received_timestamp;
  rmw_request_id_t request_id;
} rmw_service_info_t;
#endif /* !RMW_CONNEXT_HAVE_SERVICE_INFO */

rcutils_ret_t
rcutils_uint8_array_copy(
  rcutils_uint8_array_t * const dst,
  const rcutils_uint8_array_t * const src);

rmw_qos_policy_kind_t
dds_qos_policy_to_rmw_qos_policy(const DDS_QosPolicyId_t last_policy_id);

bool
ros_event_for_reader(const rmw_event_type_t ros);

DDS_StatusKind
ros_event_to_dds(const rmw_event_type_t ros, bool * const invalid);

const char *
dds_event_to_str(const DDS_StatusKind event);

/******************************************************************************
 * WaitSet: wrapper implementation on top of DDS_WaitSet
 ******************************************************************************/

class RMW_Connext_Publisher;
class RMW_Connext_Subscriber;
class RMW_Connext_Client;
class RMW_Connext_Service;
class RMW_Connext_Condition;
class RMW_Connext_GuardCondition;

class RMW_Connext_WaitSet
{
public:
  RMW_Connext_WaitSet()
  : waiting(false),
    waitset(nullptr)
  {
    if (!DDS_ConditionSeq_initialize(&this->active_conditions)) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to initialize condition sequence")
      throw std::runtime_error("failed to initialize condition sequence");
    }

    this->waitset = DDS_WaitSet_new();
    if (nullptr == this->waitset) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to create DDS waitset")
      throw std::runtime_error("failed to create DDS waitset");
    }
  }

  ~RMW_Connext_WaitSet()
  {
    if (!DDS_ConditionSeq_finalize(&this->active_conditions)) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to finalize condition sequence")
    }

    if (nullptr != this->waitset) {
      if (DDS_RETCODE_OK != DDS_WaitSet_delete(this->waitset)) {
        RMW_CONNEXT_LOG_ERROR_SET("failed to finalize DDS waitset")
      }
    }
  }

  rmw_ret_t
  wait(
    rmw_subscriptions_t * const subs,
    rmw_guard_conditions_t * const gcs,
    rmw_services_t * const srvs,
    rmw_clients_t * const cls,
    rmw_events_t * const evs,
    const rmw_time_t * const wait_timeout);

protected:
  rmw_ret_t
  attach(
    rmw_subscriptions_t * const subs,
    rmw_guard_conditions_t * const gcs,
    rmw_services_t * const srvs,
    rmw_clients_t * const cls,
    rmw_events_t * const evs);

  rmw_ret_t
  detach();

  rmw_ret_t
  process_wait(
    rmw_subscriptions_t * const subs,
    rmw_guard_conditions_t * const gcs,
    rmw_services_t * const srvs,
    rmw_clients_t * const cls,
    rmw_events_t * const evs,
    size_t & active_conditions);

  template<typename T>
  bool
  require_attach(
    const std::vector<T *> & attached_els,
    const size_t new_els_count,
    void ** const new_els);

  bool
  active_condition(RMW_Connext_Condition * const cond);

  std::mutex mutex_internal;
  bool waiting;
  DDS_WaitSet * waitset;
  DDS_ConditionSeq active_conditions;

  std::vector<RMW_Connext_Subscriber *> attached_subscribers;
  std::vector<RMW_Connext_GuardCondition *> attached_conditions;
  std::vector<RMW_Connext_Client *> attached_clients;
  std::vector<RMW_Connext_Service *> attached_services;
  std::vector<rmw_event_t *> attached_events;
};

class RMW_Connext_Condition
{
public:
  static
  rmw_ret_t
  attach(
    DDS_WaitSet * const waitset,
    DDS_Condition * const dds_condition)
  {
    if (DDS_RETCODE_OK != DDS_WaitSet_attach_condition(waitset, dds_condition)) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to attach condition to waitset")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  static
  rmw_ret_t
  detach(
    DDS_WaitSet * const waitset,
    DDS_Condition * const dds_condition)
  {
    if (DDS_RETCODE_OK != DDS_WaitSet_detach_condition(waitset, dds_condition)) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to detach condition from waitset")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  virtual bool owns(DDS_Condition * const cond) = 0;
};

class RMW_Connext_GuardCondition : public RMW_Connext_Condition
{
public:
  RMW_Connext_GuardCondition()
  : gcond(DDS_GuardCondition_new())
  {
    if (nullptr == this->gcond) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to allocate dds guard condition")
      throw std::runtime_error("failed to allocate dds guard condition");
    }
  }

  virtual ~RMW_Connext_GuardCondition()
  {
    if (nullptr != this->gcond) {
      if (DDS_RETCODE_OK != DDS_GuardCondition_delete(this->gcond)) {
        RMW_CONNEXT_LOG_ERROR_SET("failed to finalize DDS guard condition")
      }
    }
  }

  virtual bool
  owns(DDS_Condition * const cond)
  {
    return cond == DDS_GuardCondition_as_condition(this->gcond);
  }

  rmw_ret_t
  trigger()
  {
    if (DDS_RETCODE_OK !=
      DDS_GuardCondition_set_trigger_value(this->gcond, DDS_BOOLEAN_TRUE))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to trigger guard condition")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  rmw_ret_t
  reset()
  {
    if (DDS_RETCODE_OK !=
      DDS_GuardCondition_set_trigger_value(this->gcond, DDS_BOOLEAN_FALSE))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to reset guard condition")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  rmw_ret_t
  attach(DDS_WaitSet * const waitset)
  {
    return RMW_Connext_Condition::attach(
      waitset, DDS_GuardCondition_as_condition(this->gcond));
  }

  rmw_ret_t
  detach(DDS_WaitSet * const waitset)
  {
    return RMW_Connext_Condition::detach(
      waitset, DDS_GuardCondition_as_condition(this->gcond));
  }

protected:
  DDS_GuardCondition * gcond;
};

class RMW_Connext_StatusCondition : public RMW_Connext_Condition
{
public:
  explicit RMW_Connext_StatusCondition(
    DDS_Entity * const entity)
  : entity(entity),
    scond(nullptr)
  {
    if (nullptr == this->entity) {
      RMW_CONNEXT_LOG_ERROR_SET("invalid DDS entity")
      throw new std::runtime_error("invalid DDS entity");
    }

    this->scond = DDS_Entity_get_statuscondition(this->entity);
    if (nullptr == this->scond) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get DDS entity's condition")
      throw new std::runtime_error("failed to get DDS entity's condition");
    }
  }

  rmw_ret_t
  reset()
  {
    if (DDS_RETCODE_OK !=
      DDS_StatusCondition_set_enabled_statuses(
        this->scond, DDS_STATUS_MASK_NONE))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to reset status condition's statuses")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  virtual rmw_ret_t
  attach(DDS_WaitSet * const waitset, const DDS_StatusMask statuses)
  {
    DDS_StatusMask current_statuses =
      DDS_StatusCondition_get_enabled_statuses(this->scond);
    current_statuses |= statuses;
    if (DDS_RETCODE_OK !=
      DDS_StatusCondition_set_enabled_statuses(this->scond, current_statuses))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to set status condition's statuses")
      return RMW_RET_ERROR;
    }

    return RMW_Connext_Condition::attach(
      waitset, DDS_StatusCondition_as_condition(this->scond));
  }

  virtual rmw_ret_t
  detach(DDS_WaitSet * const waitset)
  {
    return RMW_Connext_Condition::detach(
      waitset, DDS_StatusCondition_as_condition(this->scond));
  }

  virtual bool
  owns(DDS_Condition * const cond)
  {
    return cond == DDS_StatusCondition_as_condition(this->scond);
  }

  bool
  has_status(const rmw_event_type_t event_type)
  {
    const DDS_StatusMask status_mask = ros_event_to_dds(event_type, nullptr);
    return DDS_Entity_get_status_changes(this->entity) & status_mask;
  }

  virtual rmw_ret_t
  get_status(const rmw_event_type_t event_type, void * const event_info) = 0;

protected:
  DDS_Entity * entity;
  DDS_StatusCondition * scond;
};

class RMW_Connext_PublisherStatusCondition : public RMW_Connext_StatusCondition
{
public:
  explicit RMW_Connext_PublisherStatusCondition(DDS_DataWriter * const writer)
  : RMW_Connext_StatusCondition(DDS_DataWriter_as_entity(writer)),
    writer(writer)
  {}

  virtual rmw_ret_t
  get_status(const rmw_event_type_t event_type, void * const event_info);

protected:
  DDS_DataWriter * const writer;
};

class RMW_Connext_SubscriberStatusCondition : public RMW_Connext_StatusCondition
{
public:
  RMW_Connext_SubscriberStatusCondition(
    DDS_DataReader * const reader,
    const bool ignore_local)
  : RMW_Connext_StatusCondition(DDS_DataReader_as_entity(reader)),
    ignore_local(ignore_local),
    participant_handle(
      DDS_Entity_get_instance_handle(
        DDS_DomainParticipant_as_entity(
          DDS_Subscriber_get_participant(
            DDS_DataReader_get_subscriber(reader))))),
    reader(reader),
    dcond(nullptr)
  {
    this->dcond = DDS_GuardCondition_new();
    if (nullptr == this->dcond) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to create reader's data condition")
      throw std::runtime_error("failed to create reader's data condition");
    }

    if (RMW_RET_OK != this->install()) {
      RMW_CONNEXT_LOG_ERROR("failed to install condition on reader")
      throw std::runtime_error("failed to install condition on reader");
    }
  }

  virtual ~RMW_Connext_SubscriberStatusCondition()
  {
    if (nullptr != this->dcond) {
      if (DDS_RETCODE_OK != DDS_GuardCondition_delete(this->dcond)) {
        RMW_CONNEXT_LOG_ERROR_SET("failed to delete reader's data condition")
      }
    }
  }

  virtual rmw_ret_t
  get_status(const rmw_event_type_t event_type, void * const event_info);

  virtual bool
  owns(DDS_Condition * const cond)
  {
    return RMW_Connext_StatusCondition::owns(cond) ||
           cond == DDS_GuardCondition_as_condition(this->dcond);
  }

  virtual rmw_ret_t
  attach(DDS_WaitSet * const waitset, const DDS_StatusMask statuses)
  {
    rmw_ret_t rc = RMW_Connext_StatusCondition::attach(waitset, statuses);
    if (RMW_RET_OK != rc) {
      return rc;
    }
    if (statuses & DDS_DATA_AVAILABLE_STATUS) {
      return RMW_Connext_Condition::attach(
        waitset, DDS_GuardCondition_as_condition(this->dcond));
    }
    return RMW_RET_OK;
  }

  virtual rmw_ret_t
  detach(DDS_WaitSet * const waitset)
  {
    rmw_ret_t rc = RMW_Connext_StatusCondition::detach(waitset);
    if (RMW_RET_OK != rc) {
      return rc;
    }
    return RMW_Connext_Condition::detach(
      waitset, DDS_GuardCondition_as_condition(this->dcond));
  }

  rmw_ret_t
  set_data_available(const bool available)
  {
    if (DDS_RETCODE_OK !=
      DDS_GuardCondition_set_trigger_value(this->dcond, available))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to set reader's data condition trigger")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  const bool ignore_local;
  const DDS_InstanceHandle_t participant_handle;

protected:
  rmw_ret_t
  install();

  DDS_DataReader * const reader;
  DDS_GuardCondition * dcond;
};

/******************************************************************************
 * Node support
 ******************************************************************************/
class RMW_Connext_Node
{
  rmw_context_impl_t * ctx;

  explicit RMW_Connext_Node(rmw_context_impl_t * const ctx)
  : ctx(ctx)
  {}

public:
  static
  RMW_Connext_Node *
  create(rmw_context_impl_t * const ctx);

  rmw_ret_t
  finalize();

  rmw_guard_condition_t *
  graph_guard_condition()
  {
    return this->ctx->common.graph_guard_condition;
  }
};


/******************************************************************************
 * Publication support
 ******************************************************************************/

class RMW_Connext_Publisher
{
public:
  static
  RMW_Connext_Publisher *
    create(
    rmw_context_impl_t * const ctx,
    DDS_DomainParticipant * const dp,
    DDS_Publisher * const pub,
    const rosidl_message_type_support_t * const type_supports,
    const char * const topic_name,
    const rmw_qos_profile_t * const qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS_PUBSUB
    const rmw_publisher_options_t * const publisher_options,
#endif /* RMW_CONNEXT_HAVE_OPTIONS_PUBSUB */
    const bool internal = false,
    const RMW_Connext_MessageType msg_type = RMW_CONNEXT_MESSAGE_USERDATA,
    const void * const intro_members = nullptr,
    const bool intro_members_cpp = false,
    std::string * const type_name = nullptr);

  rmw_ret_t
  finalize();

  DDS_DataWriter *
  writer() const
  {
    return this->dds_writer;
  }

  RMW_Connext_MessageTypeSupport *
  message_type_support() const
  {
    return this->type_support;
  }

  DDS_InstanceHandle_t
  instance_handle()
  {
    return DDS_Entity_get_instance_handle(
      DDS_DataWriter_as_entity(this->writer()));
  }

  rmw_ret_t
  write(
    const void * const ros_message,
    const bool serialized,
    int64_t * const sn_out = nullptr);

  rmw_ret_t
  enable() const
  {
    if (DDS_RETCODE_OK !=
      DDS_Entity_enable(
        DDS_Topic_as_entity(
          DDS_DataWriter_get_topic(this->dds_writer))))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to enable dds writer's topic")
      return RMW_RET_ERROR;
    }

    if (DDS_RETCODE_OK !=
      DDS_Entity_enable(DDS_DataWriter_as_entity(this->dds_writer)))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to enable dds writer")
      return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
  }

  RMW_Connext_PublisherStatusCondition *
  condition()
  {
    return &this->status_condition;
  }

  const rmw_gid_t * gid() const
  {
    return &this->ros_gid;
  }

  size_t
  subscriptions_count();

  rmw_ret_t
  assert_liveliness();

  rmw_ret_t
  qos(rmw_qos_profile_t * const qos);

  rmw_ret_t
  requestreply_header_to_dds(
    const RMW_Connext_RequestReplyMessage * const rr_msg,
    DDS_SampleIdentity_t * const sample_identity,
    DDS_SampleIdentity_t * const related_sample_identity);

  DDS_Topic * dds_topic()
  {
    return DDS_DataWriter_get_topic(this->dds_writer);
  }

  DDS_Publisher * dds_publisher()
  {
    return DDS_DataWriter_get_publisher(this->dds_writer);
  }

  DDS_DomainParticipant * dds_participant()
  {
    DDS_Publisher * const pub = this->dds_publisher();

    return DDS_Publisher_get_participant(pub);
  }

private:
  rmw_context_impl_t * ctx;
  DDS_DataWriter * dds_writer;
  RMW_Connext_MessageTypeSupport * type_support;
  const bool created_topic;
  rmw_gid_t ros_gid;
  RMW_Connext_PublisherStatusCondition status_condition;

  RMW_Connext_Publisher(
    rmw_context_impl_t * const ctx,
    DDS_DataWriter * const dds_writer,
    RMW_Connext_MessageTypeSupport * const type_support,
    const bool created_topic);
};


rmw_publisher_t *
  rmw_connextdds_create_publisher(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  DDS_DomainParticipant * const dp,
  DDS_Publisher * const pub,
  const rosidl_message_type_support_t * const type_supports,
  const char * const topic_name,
  const rmw_qos_profile_t * const qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS_PUBSUB
  const rmw_publisher_options_t * const publisher_options,
#endif /* RMW_CONNEXT_HAVE_OPTIONS_PUBSUB */
  const bool internal = false);

rmw_ret_t
rmw_connextdds_destroy_publisher(
  rmw_context_impl_t * const ctx,
  rmw_publisher_t * const pub);

/******************************************************************************
 * Subscription support
 ******************************************************************************/

class RMW_Connext_Subscriber
{
public:
  static
  RMW_Connext_Subscriber *
    create(
    rmw_context_impl_t * const ctx,
    DDS_DomainParticipant * const dp,
    DDS_Subscriber * const sub,
    const rosidl_message_type_support_t * const type_supports,
    const char * const topic_name,
    const rmw_qos_profile_t * const qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS_PUBSUB
    const rmw_subscription_options_t * const subscriber_options,
#else
    const bool ignore_local_publications,
#endif /* RMW_CONNEXT_HAVE_OPTIONS_PUBSUB */
    const bool internal = false,
    const RMW_Connext_MessageType msg_type = RMW_CONNEXT_MESSAGE_USERDATA,
    const void * const intro_members = nullptr,
    const bool intro_members_cpp = false,
    std::string * const type_name = nullptr,
    const char * const cft_name = nullptr,
    const char * const cft_filter = nullptr);

  rmw_ret_t
  finalize();

  DDS_DataReader *
  reader() const
  {
    return this->dds_reader;
  }

  RMW_Connext_UntypedSampleSeq *
  data_seq()
  {
    return &this->loan_data;
  }
  DDS_SampleInfoSeq *
  info_seq()
  {
    return &this->loan_info;
  }

  RMW_Connext_MessageTypeSupport *
  message_type_support() const
  {
    return this->type_support;
  }

  DDS_InstanceHandle_t
  instance_handle()
  {
    return DDS_Entity_get_instance_handle(
      DDS_DataReader_as_entity(this->reader()));
  }

  DDS_InstanceHandle_t
  participant_instance_handle()
  {
    return DDS_Entity_get_instance_handle(
      DDS_DomainParticipant_as_entity(
        DDS_Subscriber_get_participant(
          DDS_DataReader_get_subscriber(this->reader()))));
  }

  rmw_ret_t
  enable() const
  {
    if (DDS_RETCODE_OK !=
      DDS_Entity_enable(DDS_Topic_as_entity(this->dds_topic)))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to enable dds reader's topic")
      return RMW_RET_ERROR;
    }

    if (DDS_RETCODE_OK !=
      DDS_Entity_enable(DDS_DataReader_as_entity(this->dds_reader)))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to enable dds reader")
      return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
  }

  RMW_Connext_SubscriberStatusCondition *
  condition()
  {
    return &this->status_condition;
  }

  const rmw_gid_t * gid() const
  {
    return &this->ros_gid;
  }

  size_t
  publications_count();

  rmw_ret_t
  qos(rmw_qos_profile_t * const qos);

  rmw_ret_t
  loan_messages();

  rmw_ret_t
  return_messages();

  rmw_ret_t
  loan_messages_if_needed()
  {
    rmw_ret_t rc = RMW_RET_OK;

    // take messages from reader if we don't have an outstanding loan
    // or if we have consumed the current loan
    if (this->loan_len == 0 || this->loan_next >= this->loan_len) {
      /* return any previously loaned messages */
      if (this->loan_len > 0) {
        rc = this->return_messages();
        if (RMW_RET_OK != rc) {
          return rc;
        }
      }
      /* loan messages from reader */
      rc = this->loan_messages();
      if (RMW_RET_OK != rc) {
        return rc;
      }
    }

    return RMW_RET_OK;
  }

  void
  requestreply_header_from_dds(
    RMW_Connext_RequestReplyMessage * const rr_msg,
    const DDS_SampleIdentity_t * const sample_identity,
    const DDS_SampleIdentity_t * const related_sample_identity);

  rmw_ret_t
  take_next(
    void ** const ros_messages,
    rmw_message_info_t * const message_infos,
    const size_t max_samples,
    size_t * const taken,
    const bool serialized,
    const DDS_InstanceHandle_t * const request_writer_handle = nullptr);

  rmw_ret_t
  take_message(
    void * const ros_message,
    rmw_message_info_t * const message_info,
    bool * const taken,
    const DDS_InstanceHandle_t * const request_writer_handle = nullptr);

#if RMW_CONNEXT_HAVE_TAKE_SEQ
  rmw_ret_t
  take(
    rmw_message_sequence_t * const message_sequence,
    rmw_message_info_sequence_t * const message_info_sequence,
    const size_t max_samples,
    size_t * const taken);
#endif /* RMW_CONNEXT_HAVE_TAKE_SEQ */

  rmw_ret_t
  take_serialized(
    rmw_serialized_message_t * const serialized_message,
    rmw_message_info_t * const message_info,
    bool * const taken);

  bool
  has_data()
  {
    std::lock_guard<std::mutex> lock(this->loan_mutex);
    if (RMW_RET_OK != this->loan_messages_if_needed()) {
      RMW_CONNEXT_LOG_ERROR("failed to check loaned messages")
      return false;
    }
    bool has_data = this->loan_len > 0;
    RMW_CONNEXT_ASSERT(!has_data || this->loan_next < this->loan_len)
    return has_data;
  }

  DDS_Subscriber * dds_subscriber()
  {
    return DDS_DataReader_get_subscriber(this->dds_reader);
  }

  DDS_DomainParticipant * dds_participant()
  {
    DDS_Subscriber * const sub = this->dds_subscriber();

    return DDS_Subscriber_get_participant(sub);
  }

  DDS_Topic * topic()
  {
    return this->dds_topic;
  }

private:
  rmw_context_impl_t * ctx;
  DDS_DataReader * dds_reader;
  DDS_Topic * dds_topic;
  DDS_TopicDescription * dds_topic_cft;
  RMW_Connext_MessageTypeSupport * type_support;
  rmw_gid_t ros_gid;
  const bool created_topic;
  RMW_Connext_SubscriberStatusCondition status_condition;
  RMW_Connext_UntypedSampleSeq loan_data;
  DDS_SampleInfoSeq loan_info;
  size_t loan_len;
  size_t loan_next;
  std::mutex loan_mutex;
  bool internal;

  RMW_Connext_Subscriber(
    rmw_context_impl_t * const ctx,
    DDS_DataReader * const dds_reader,
    DDS_Topic * const dds_topic,
    RMW_Connext_MessageTypeSupport * const type_support,
    const bool ignore_local,
    const bool created_topic,
    DDS_TopicDescription * const dds_topic_cft,
    const bool internal);

  // friend class RMW_Connext_SubscriberStatusCondition;
};

rmw_subscription_t *
  rmw_connextdds_create_subscriber(
  rmw_context_impl_t * const ctx,
  const rmw_node_t * const node,
  DDS_DomainParticipant * const dp,
  DDS_Subscriber * const sub,
  const rosidl_message_type_support_t * const type_supports,
  const char * const topic_name,
  const rmw_qos_profile_t * const qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS_PUBSUB
  const rmw_subscription_options_t * const subscriber_options,
#else
  const bool ignore_local_publications,
#endif /* RMW_CONNEXT_HAVE_OPTIONS_PUBSUB */
  const bool internal = false);

rmw_ret_t
rmw_connextdds_destroy_subscriber(
  rmw_context_impl_t * const ctx,
  rmw_subscription_t * const sub);

void
rmw_connextdds_message_info_from_dds(
  rmw_message_info_t * const to,
  const DDS_SampleInfo * const from);

/******************************************************************************
 * Client/Service support
 ******************************************************************************/

class RMW_Connext_Client
{
  RMW_Connext_Publisher * request_pub;
  RMW_Connext_Subscriber * reply_sub;
#if RMW_CONNEXT_EMULATE_REQUESTREPLY
  std::atomic_uint next_request_id;
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */

  RMW_Connext_Client()
  : request_pub(nullptr),
    reply_sub(nullptr)
#if RMW_CONNEXT_EMULATE_REQUESTREPLY
    ,
    next_request_id(1)
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */
  {}

public:
  static
  RMW_Connext_Client *
  create(
    rmw_context_impl_t * const ctx,
    DDS_DomainParticipant * const dp,
    DDS_Publisher * const pub,
    DDS_Subscriber * const sub,
    const rosidl_service_type_support_t * const type_supports,
    const char * const svc_name,
    const rmw_qos_profile_t * const qos_policies);

  rmw_ret_t
  finalize();

  rmw_ret_t
  is_service_available(bool & available);

  rmw_ret_t
  take_response(
    rmw_service_info_t * const request_header,
    void * const ros_response,
    bool * const taken);

  rmw_ret_t
  send_request(
    const void * const ros_request,
    int64_t * const sequence_id);

  RMW_Connext_Publisher *
  publisher() const
  {
    return this->request_pub;
  }

  RMW_Connext_Subscriber *
  subscriber() const
  {
    return this->reply_sub;
  }

  rmw_ret_t
  enable();
};

class RMW_Connext_Service
{
  RMW_Connext_Publisher * reply_pub;
  RMW_Connext_Subscriber * request_sub;

public:
  static
  RMW_Connext_Service *
  create(
    rmw_context_impl_t * const ctx,
    DDS_DomainParticipant * const dp,
    DDS_Publisher * const pub,
    DDS_Subscriber * const sub,
    const rosidl_service_type_support_t * const type_supports,
    const char * const svc_name,
    const rmw_qos_profile_t * const qos_policies);

  rmw_ret_t
  finalize();

  rmw_ret_t
  take_request(
    rmw_service_info_t * const request_header,
    void * const ros_request,
    bool * const taken);

  rmw_ret_t
  send_response(
    rmw_request_id_t * const request_id,
    const void * const ros_response);

  RMW_Connext_Publisher *
  publisher() const
  {
    return this->reply_pub;
  }

  RMW_Connext_Subscriber *
  subscriber() const
  {
    return this->request_sub;
  }

  rmw_ret_t
  enable();
};


/******************************************************************************
 * Event support
 ******************************************************************************/

class RMW_Connext_Event
{
public:
  static
  RMW_Connext_StatusCondition *
  condition(const rmw_event_t * const event)
  {
    if (RMW_Connext_Event::reader_event(event)) {
      return RMW_Connext_Event::subscriber(event)->condition();
    } else {
      return RMW_Connext_Event::publisher(event)->condition();
    }
  }

  static
  bool
  active(rmw_event_t * const event)
  {
    RMW_Connext_StatusCondition * condition = nullptr;
    if (RMW_Connext_Event::reader_event(event)) {
      condition = RMW_Connext_Event::subscriber(event)->condition();
    } else {
      condition = RMW_Connext_Event::publisher(event)->condition();
    }
    return condition->has_status(event->event_type);
  }

  static
  bool
  writer_event(const rmw_event_t * const event)
  {
    return !ros_event_for_reader(event->event_type);
  }

  static
  bool
  reader_event(const rmw_event_t * const event)
  {
    return ros_event_for_reader(event->event_type);
  }

  static
  RMW_Connext_Publisher *
  publisher(const rmw_event_t * const event)
  {
    return reinterpret_cast<RMW_Connext_Publisher *>(event->data);
  }

  static
  RMW_Connext_Subscriber *
  subscriber(const rmw_event_t * const event)
  {
    return reinterpret_cast<RMW_Connext_Subscriber *>(event->data);
  }
};


/******************************************************************************
 * Waitsets and Conditions
 ******************************************************************************/

rmw_guard_condition_t *
rmw_connextdds_create_guard_condition();

rmw_ret_t
rmw_connextdds_destroy_guard_condition(rmw_guard_condition_t * const gc);

rmw_ret_t
rmw_connextdds_trigger_guard_condition(const rmw_guard_condition_t * const gc);

rmw_wait_set_t *
rmw_connextdds_create_waitset(const size_t max_conditions);

rmw_ret_t
rmw_connextdds_destroy_waitset(rmw_wait_set_t * const ws);

rmw_ret_t
rmw_connextdds_waitset_wait(
  rmw_wait_set_t * const ws,
  rmw_subscriptions_t * subs,
  rmw_guard_conditions_t * gcs,
  rmw_services_t * srvs,
  rmw_clients_t * cls,
  rmw_events_t * evs,
  const rmw_time_t * wait_timeout);

/******************************************************************************
 * GUID helpers
 ******************************************************************************/

rmw_ret_t
rmw_connextdds_gid_to_guid(const rmw_gid_t & gid, struct DDS_GUID_t & guid);

rmw_ret_t
rmw_connextdds_guid_to_gid(const struct DDS_GUID_t & guid, rmw_gid_t & gid);

void
rmw_connextdds_sn_ros_to_dds(
  const int64_t sn_ros, struct DDS_SequenceNumber_t & sn_dds);

#define rmw_connextdds_sn_ros_to_dds(sn_ros_, sn_dds_) \
  { \
    (sn_dds_).high = ((sn_ros_) & 0xFFFFFFFF00000000LL) >> 32; \
    (sn_dds_).low = (sn_ros_) & 0x00000000FFFFFFFFLL; \
  }

void
rmw_connextdds_sn_dds_to_ros(
  const struct DDS_SequenceNumber_t & sn_dds, int64_t & sn_ros);

#define rmw_connextdds_sn_dds_to_ros(sn_dds_, sn_ros_) \
  { \
    sn_ros_ = ((int64_t)(sn_dds_).high) << 32 | (sn_dds_).low; \
  }

void rmw_connextdds_ih_to_gid(
  const DDS_InstanceHandle_t & ih, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_Entity * const entity, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_DomainParticipant * const dp, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_Publisher * const pub, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_Subscriber * const sub, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_DataWriter * const writer, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_DataReader * const reader, rmw_gid_t & gid);

void rmw_connextdds_get_entity_gid(
  DDS_Topic * const topic, rmw_gid_t & gid);

/******************************************************************************
 * Type helpers
 ******************************************************************************/
std::string
rmw_connextdds_create_type_name(
  const message_type_support_callbacks_t * callbacks,
  const bool mangle_names = true);

#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT

std::string
rmw_connextdds_create_type_name(
  const rosidl_typesupport_introspection_cpp::MessageMembers * const members,
  const bool mangle_names = true);

std::string
rmw_connextdds_create_type_name(
  const rosidl_typesupport_introspection_c__MessageMembers * const members,
  const bool mangle_names = true);

#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */

std::string
rmw_connextdds_create_type_name_request(
  const service_type_support_callbacks_t * callbacks,
  const bool mangle_names = true);

std::string
rmw_connextdds_create_type_name_response(
  const service_type_support_callbacks_t * callbacks,
  const bool mangle_names = true);

/******************************************************************************
 * Topic Helpers
 ******************************************************************************/
std::string
rmw_connextdds_create_topic_name(
  const char * prefix,
  const char * topic_name,
  const char * suffix,
  bool avoid_ros_namespace_conventions);

std::string
rmw_connextdds_create_topic_name(
  const char * prefix,
  const char * topic_name,
  const char * suffix,
  const rmw_qos_profile_t * qos_policies);

/******************************************************************************
 * Qos Helpers
 ******************************************************************************/

rmw_ret_t
  rmw_connextdds_get_readerwriter_qos(
  const bool writer_qos,
  RMW_Connext_MessageTypeSupport * const type_support,
  DDS_HistoryQosPolicy * const history,
  DDS_ReliabilityQosPolicy * const reliability,
  DDS_DurabilityQosPolicy * const durability,
  DDS_DeadlineQosPolicy * const deadline,
  DDS_LivelinessQosPolicy * const liveliness,
  DDS_ResourceLimitsQosPolicy * const resource_limits,
  DDS_PublishModeQosPolicy * const publish_mode,
#if RMW_CONNEXT_HAVE_LIFESPAN_QOS
  DDS_LifespanQosPolicy * const lifespan,
#endif /* RMW_CONNEXT_HAVE_LIFESPAN_QOS */
  const rmw_qos_profile_t * const qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS_PUBSUB
  ,
  const rmw_publisher_options_t * const pub_options,
  const rmw_subscription_options_t * const sub_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS_PUBSUB */
  );

rmw_ret_t
  rmw_connextdds_readerwriter_qos_to_ros(
  const DDS_HistoryQosPolicy * const history,
  const DDS_ReliabilityQosPolicy * const reliability,
  const DDS_DurabilityQosPolicy * const durability,
  const DDS_DeadlineQosPolicy * const deadline,
  const DDS_LivelinessQosPolicy * const liveliness,
#if RMW_CONNEXT_HAVE_LIFESPAN_QOS
  const DDS_LifespanQosPolicy * const lifespan,
#endif /* RMW_CONNEXT_HAVE_LIFESPAN_QOS */
  rmw_qos_profile_t * const qos_policies);

bool
rmw_connextdds_find_string_in_list(
  const DDS_StringSeq * const profile_names,
  const char * const profile);

rmw_ret_t
rmw_connextdds_list_context_qos_profiles(
  rmw_context_impl_t * const ctx,
  std::vector<std::string> & profiles);

rmw_ret_t
rmw_connextdds_list_publisher_qos_profiles(
  rmw_context_impl_t * const ctx,
  const char * const topic_name,
  std::vector<std::string> & profiles);

rmw_ret_t
rmw_connextdds_list_subscription_qos_profiles(
  rmw_context_impl_t * const ctx,
  const char * const topic_name,
  std::vector<std::string> & profiles);

rmw_ret_t
rmw_connextdds_list_client_qos_profiles(
  rmw_context_impl_t * const ctx,
  const char * const service_name,
  std::vector<std::string> & req_profiles,
  std::vector<std::string> & rep_profiles);

rmw_ret_t
rmw_connextdds_list_service_qos_profiles(
  rmw_context_impl_t * const ctx,
  const char * const service_name,
  std::vector<std::string> & req_profiles,
  std::vector<std::string> & rep_profiles);

#endif  // RMW_CONNEXTDDS__RMW_IMPL_HPP_
