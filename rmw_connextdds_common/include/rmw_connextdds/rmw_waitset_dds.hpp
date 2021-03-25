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

#ifndef RMW_CONNEXTDDS__RMW_WAITSET_DDS_HPP_
#define RMW_CONNEXTDDS__RMW_WAITSET_DDS_HPP_

#include <vector>
#include <map>

#include "rmw_connextdds/context.hpp"

#if !RMW_CONNEXT_CPP_STD_WAITSETS
/******************************************************************************
 * WaitSet: wrapper implementation on top of DDS_WaitSet
 ******************************************************************************/
class RMW_Connext_Condition;
class RMW_Connext_GuardCondition;
class RMW_Connext_StatusCondition;
class RMW_Connext_SubscriberStatusCondition;
class RMW_Connext_PublisherStatusCondition;

enum RMW_Connext_WaitSetState
{
  RMW_CONNEXT_WAITSET_FREE,
  RMW_CONNEXT_WAITSET_ACQUIRING,
  RMW_CONNEXT_WAITSET_BLOCKED,
  RMW_CONNEXT_WAITSET_RELEASING,
  RMW_CONNEXT_WAITSET_INVALIDATING
};

class RMW_Connext_WaitSet
{
public:
  RMW_Connext_WaitSet()
  : state(RMW_CONNEXT_WAITSET_FREE),
    waitset(nullptr)
  {
    if (!DDS_ConditionSeq_initialize(&this->active_conditions)) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to initialize condition sequence")
      throw std::runtime_error("failed to initialize condition sequence");
    }

    RMW_Connext_WaitSet * const ws = this;
    auto scope_exit = rcpputils::make_scope_exit(
      [ws]() {
        if (DDS_RETCODE_OK != DDS_ConditionSeq_finalize(&ws->active_conditions)) {
          RMW_CONNEXT_LOG_ERROR("failed to finalize condition sequence")
        }
        if (nullptr != ws->waitset &&
        DDS_RETCODE_OK != DDS_WaitSet_delete(ws->waitset))
        {
          RMW_CONNEXT_LOG_ERROR("failed to finalize waitset")
        }
      });

    this->waitset = DDS_WaitSet_new();
    if (nullptr == this->waitset) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to create DDS waitset")
      throw std::runtime_error("failed to create DDS waitset");
    }

    scope_exit.cancel();
  }

  ~RMW_Connext_WaitSet()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_internal);
      // the waiset should be accessed while being deleted, otherwise
      // bad things(tm) will happen.
      if (RMW_CONNEXT_WAITSET_FREE != this->state) {
        RMW_CONNEXT_LOG_ERROR_SET("deleting a waitset while waiting on it")
      }
      if (RMW_RET_OK != this->detach()) {
        RMW_CONNEXT_LOG_ERROR_SET("failed to detach conditions from deleted waitset")
      }
    }
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

  rmw_ret_t
  invalidate(RMW_Connext_Condition * const condition);

protected:
  bool
  is_attached(RMW_Connext_Condition * const condition);

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
  std::condition_variable state_cond;
  RMW_Connext_WaitSetState state;
  DDS_WaitSet * waitset;
  DDS_ConditionSeq active_conditions;

  std::vector<RMW_Connext_Subscriber *> attached_subscribers;
  std::vector<RMW_Connext_GuardCondition *> attached_conditions;
  std::vector<RMW_Connext_Client *> attached_clients;
  std::vector<RMW_Connext_Service *> attached_services;
  std::vector<rmw_event_t *> attached_events;
  std::map<rmw_event_t *, rmw_event_t> attached_events_cache;

  friend class RMW_Connext_Condition;
  friend class RMW_Connext_GuardCondition;
  friend class RMW_Connext_StatusCondition;
  friend class RMW_Connext_SubscriberStatusCondition;
  friend class RMW_Connext_PublisherStatusCondition;
};

class RMW_Connext_Condition
{
public:
  RMW_Connext_Condition()
  : attached_waitset(nullptr),
    deleted(false)
  {
  }

  void
  invalidate()
  {
    RMW_Connext_WaitSet * attached_waitset = nullptr;
    {
      std::lock_guard<std::mutex> lock(this->mutex_internal);
      // This function might have already been called by a derived class
      if (this->deleted) {
        return;
      }
      this->deleted = true;
      attached_waitset = this->attached_waitset;
    }
    if (nullptr != this->attached_waitset) {
      attached_waitset->invalidate(this);
    }
  }

protected:
  static rmw_ret_t
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

  static rmw_ret_t
  detach(
    DDS_WaitSet * const waitset,
    DDS_Condition * const dds_condition)
  {
    // detach_condition() returns BAD_PARAMETER if the condition is not attached
    DDS_ReturnCode_t rc = DDS_WaitSet_detach_condition(waitset, dds_condition);
    if (DDS_RETCODE_OK != rc &&
      DDS_RETCODE_BAD_PARAMETER != rc &&
      DDS_RETCODE_PRECONDITION_NOT_MET != rc)
    {
      RMW_CONNEXT_LOG_ERROR_A_SET(
        "failed to detach condition from waitset: %d", rc)
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  virtual bool owns(DDS_Condition * const cond) = 0;

  rmw_ret_t
  attach(RMW_Connext_WaitSet * const waitset)
  {
    // refuse to attach
    if (this->deleted) {
      return RMW_RET_ERROR;
    }

    rmw_ret_t rc = RMW_RET_ERROR;

    if (nullptr != this->attached_waitset) {
      if (waitset != this->attached_waitset) {
        rc = this->detach();
        if (RMW_RET_OK != rc) {
          return rc;
        }
      } else {
        // condition is already attached to this waitset, nothing to do
        return RMW_RET_OK;
      }
    }
    rc = this->_attach(waitset->waitset);
    if (RMW_RET_OK != rc) {
      return rc;
    }
    this->attached_waitset = waitset;
    return RMW_RET_OK;
  }

  rmw_ret_t
  detach()
  {
    rmw_ret_t rc = RMW_RET_ERROR;

    if (nullptr == this->attached_waitset) {
      // condition is already detached, nothing to do
      return RMW_RET_OK;
    }
    rc = this->_detach(this->attached_waitset->waitset);
    if (RMW_RET_OK != rc) {
      return rc;
    }
    this->attached_waitset = nullptr;
    return RMW_RET_OK;
  }

  virtual rmw_ret_t _attach(DDS_WaitSet * const waitset) = 0;
  virtual rmw_ret_t _detach(DDS_WaitSet * const waitset) = 0;

  RMW_Connext_WaitSet * attached_waitset;
  bool deleted;
  std::mutex mutex_internal;

  friend class RMW_Connext_WaitSet;
};

class RMW_Connext_GuardCondition : public RMW_Connext_Condition
{
public:
  explicit RMW_Connext_GuardCondition(const bool internal)
  : gcond(DDS_GuardCondition_new())
  {
    UNUSED_ARG(internal);
    if (nullptr == this->gcond) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to allocate dds guard condition")
      throw std::runtime_error("failed to allocate dds guard condition");
    }
  }

  virtual ~RMW_Connext_GuardCondition()
  {
    if (nullptr == RMW_Connext_gv_DomainParticipantFactory) {
      // DomainParticipantFactory has already been finalized.
      // Don't try to delete the guard condition, or we might
      // end up with a segfault.
#if RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY
      // For ROS2 releases > Foxy, this is unexpected behavior
      // so print an error message.
      RMW_CONNEXT_LOG_ERROR(
        "DomainParticipantFactory already finalized, "
        "leaked DDS guard condition")
#endif /* RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY */
      return;
    }
    this->invalidate();
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
  reset_trigger()
  {
    if (DDS_RETCODE_OK !=
      DDS_GuardCondition_set_trigger_value(this->gcond, DDS_BOOLEAN_FALSE))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to reset guard condition")
      return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
  }

  virtual rmw_ret_t _attach(DDS_WaitSet * const waitset)
  {
    return RMW_Connext_Condition::attach(
      waitset, DDS_GuardCondition_as_condition(this->gcond));
  }
  virtual rmw_ret_t _detach(DDS_WaitSet * const waitset)
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

  ~RMW_Connext_StatusCondition()
  {
    this->invalidate();
  }

  rmw_ret_t
  reset_statuses()
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

  rmw_ret_t
  enable_statuses(const DDS_StatusMask statuses)
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
    return RMW_RET_OK;
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

  virtual rmw_ret_t _attach(DDS_WaitSet * const waitset)
  {
    return RMW_Connext_Condition::attach(
      waitset, DDS_StatusCondition_as_condition(this->scond));
  }
  virtual rmw_ret_t _detach(DDS_WaitSet * const waitset)
  {
    return RMW_Connext_Condition::detach(
      waitset, DDS_StatusCondition_as_condition(this->scond));
  }

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
  {
  }

  ~RMW_Connext_PublisherStatusCondition()
  {
    this->invalidate();
  }

  virtual rmw_ret_t
  get_status(const rmw_event_type_t event_type, void * const event_info);

  // Helper functions to retrieve status information
  inline rmw_ret_t
  get_liveliness_lost_status(rmw_liveliness_lost_status_t * const status)
  {
    DDS_LivelinessLostStatus dds_status = DDS_LivelinessLostStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataWriter_get_liveliness_lost_status(this->writer, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get liveliness lost status")
      return RMW_RET_ERROR;
    }

    status->total_count = dds_status.total_count;
    status->total_count_change = dds_status.total_count_change;

    return RMW_RET_OK;
  }

  inline rmw_ret_t
  get_offered_deadline_missed_status(
    rmw_offered_deadline_missed_status_t * const status)
  {
    DDS_OfferedDeadlineMissedStatus dds_status = DDS_OfferedDeadlineMissedStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataWriter_get_offered_deadline_missed_status(this->writer, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get offered deadline missed status")
      return RMW_RET_ERROR;
    }

    status->total_count = dds_status.total_count;
    status->total_count_change = dds_status.total_count_change;

    return RMW_RET_OK;
  }

  inline rmw_ret_t
  get_offered_qos_incompatible_status(
    rmw_offered_qos_incompatible_event_status_t * const status)
  {
    DDS_OfferedIncompatibleQosStatus dds_status =
      DDS_OfferedIncompatibleQosStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataWriter_get_offered_incompatible_qos_status(this->writer, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get offered incompatible qos status")
      return RMW_RET_ERROR;
    }

    status->total_count = dds_status.total_count;
    status->total_count_change = dds_status.total_count_change;
    status->last_policy_kind =
      dds_qos_policy_to_rmw_qos_policy(dds_status.last_policy_id);

    return RMW_RET_OK;
  }

protected:
  DDS_DataWriter * const writer;
};

class RMW_Connext_SubscriberStatusCondition : public RMW_Connext_StatusCondition
{
public:
  RMW_Connext_SubscriberStatusCondition(
    DDS_DataReader * const reader,
    const bool ignore_local,
    const bool internal)
  : RMW_Connext_StatusCondition(DDS_DataReader_as_entity(reader)),
    ignore_local(ignore_local),
    participant_handle(
      DDS_Entity_get_instance_handle(
        DDS_DomainParticipant_as_entity(
          DDS_Subscriber_get_participant(
            DDS_DataReader_get_subscriber(reader))))),
    reader(reader),
    dcond(nullptr),
    attached_waitset_dcond(nullptr)
  {
    UNUSED_ARG(internal);
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
    this->invalidate();
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

  rmw_ret_t
  attach_data()
  {
    if (nullptr == this->attached_waitset) {
      RMW_CONNEXT_LOG_ERROR("condition must be already attached")
      return RMW_RET_ERROR;
    }
    rmw_ret_t rc = RMW_Connext_Condition::attach(
      this->attached_waitset->waitset,
      DDS_GuardCondition_as_condition(this->dcond));
    if (RMW_RET_OK != rc) {
      return rc;
    }
    this->attached_waitset_dcond = this->attached_waitset;
    return RMW_RET_OK;
  }

  virtual rmw_ret_t
  detach()
  {
    rmw_ret_t rc = RMW_Connext_Condition::detach();
    if (RMW_RET_OK != rc) {
      return rc;
    }
    if (nullptr != this->attached_waitset_dcond) {
      RMW_Connext_WaitSet * const ws = this->attached_waitset_dcond;
      this->attached_waitset_dcond = nullptr;
      return RMW_Connext_Condition::detach(
        ws->waitset, DDS_GuardCondition_as_condition(this->dcond));
    }
    return RMW_RET_OK;
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

  // Methods for "internal" subscribers only
  inline rmw_ret_t
  trigger_loan_guard_condition(const bool trigger_value)
  {
    UNUSED_ARG(trigger_value);
    return RMW_RET_OK;
  }

  // Helper functions to retrieve status information
  inline rmw_ret_t
  get_liveliness_changed_status(rmw_liveliness_changed_status_t * const status)
  {
    DDS_LivelinessChangedStatus dds_status = DDS_LivelinessChangedStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataReader_get_liveliness_changed_status(this->reader, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get liveliness changed status")
      return RMW_RET_ERROR;
    }

    status->alive_count = dds_status.alive_count;
    status->alive_count_change = dds_status.alive_count_change;
    status->not_alive_count = dds_status.not_alive_count;
    status->not_alive_count_change = dds_status.not_alive_count_change;

    return RMW_RET_OK;
  }

  inline rmw_ret_t
  get_requested_deadline_missed_status(
    rmw_requested_deadline_missed_status_t * const status)
  {
    DDS_RequestedDeadlineMissedStatus dds_status =
      DDS_RequestedDeadlineMissedStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataReader_get_requested_deadline_missed_status(this->reader, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get requested deadline missed status")
      return RMW_RET_ERROR;
    }

    status->total_count = dds_status.total_count;
    status->total_count_change = dds_status.total_count_change;

    return RMW_RET_OK;
  }

  inline rmw_ret_t
  get_requested_qos_incompatible_status(
    rmw_requested_qos_incompatible_event_status_t * const status)
  {
    DDS_RequestedIncompatibleQosStatus dds_status =
      DDS_RequestedIncompatibleQosStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataReader_get_requested_incompatible_qos_status(this->reader, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get requested incompatible qos status")
      return RMW_RET_ERROR;
    }

    status->total_count = dds_status.total_count;
    status->total_count_change = dds_status.total_count_change;
    status->last_policy_kind =
      dds_qos_policy_to_rmw_qos_policy(dds_status.last_policy_id);

    return RMW_RET_OK;
  }

#if RMW_CONNEXT_HAVE_MESSAGE_LOST
  inline rmw_ret_t
  get_message_lost_status(rmw_message_lost_status_t * const status)
  {
    DDS_SampleLostStatus dds_status = DDS_SampleLostStatus_INITIALIZER;

    if (DDS_RETCODE_OK !=
      DDS_DataReader_get_sample_lost_status(this->reader, &dds_status))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to get sample lost status")
      return RMW_RET_ERROR;
    }

    status->total_count = dds_status.total_count;
    status->total_count_change = dds_status.total_count_change;

    return RMW_RET_OK;
  }
#endif /* RMW_CONNEXT_HAVE_MESSAGE_LOST */

protected:
  rmw_ret_t
  install();

  DDS_DataReader * const reader;
  DDS_GuardCondition * dcond;
  RMW_Connext_WaitSet * attached_waitset_dcond;
};

/******************************************************************************
 * Event support
 ******************************************************************************/
class RMW_Connext_Event
{
public:
  static inline
  RMW_Connext_StatusCondition *
  condition(const rmw_event_t * const event);

  static inline
  bool
  active(rmw_event_t * const event);

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
#endif /* RMW_CONNEXT_CPP_STD_WAITSETS */
#endif  // RMW_CONNEXTDDS__RMW_WAITSET_DDS_HPP_
