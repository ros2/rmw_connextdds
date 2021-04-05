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

#include "rmw_connextdds/rmw_impl.hpp"

#if RMW_CONNEXT_CPP_STD_WAITSETS
/******************************************************************************
 * Event
 ******************************************************************************/
DDS_Condition *
RMW_Connext_Event::condition(const rmw_event_t * const event)
{
  if (RMW_Connext_Event::reader_event(event)) {
    return RMW_Connext_Event::subscriber(event)->condition()->dds_condition();
  } else {
    return RMW_Connext_Event::publisher(event)->condition()->dds_condition();
  }
}

rmw_ret_t
RMW_Connext_Event::enable(rmw_event_t * const event)
{
  if (RMW_Connext_Event::reader_event(event)) {
    return RMW_Connext_Event::subscriber(event)->condition()->enable_statuses(
      static_cast<DDS_StatusMask>(ros_event_to_dds(event->event_type, nullptr)));
  } else {
    return RMW_Connext_Event::publisher(event)->condition()->enable_statuses(
      static_cast<DDS_StatusMask>(ros_event_to_dds(event->event_type, nullptr)));
  }
}

rmw_ret_t
RMW_Connext_Event::disable(rmw_event_t * const event)
{
  if (RMW_Connext_Event::reader_event(event)) {
    return RMW_Connext_Event::subscriber(event)->condition()->disable_statuses(
      static_cast<DDS_StatusMask>(ros_event_to_dds(event->event_type, nullptr)));
  } else {
    return RMW_Connext_Event::publisher(event)->condition()->disable_statuses(
      static_cast<DDS_StatusMask>(ros_event_to_dds(event->event_type, nullptr)));
  }
}

bool
RMW_Connext_Event::active(rmw_event_t * const event)
{
  if (RMW_Connext_Event::reader_event(event)) {
    return RMW_Connext_Event::subscriber(event)->condition()->has_status(event->event_type);
  } else {
    return RMW_Connext_Event::publisher(event)->condition()->has_status(event->event_type);
  }
}

/******************************************************************************
 * StdWaitSet
 ******************************************************************************/

void
RMW_Connext_DataReaderListener_requested_deadline_missed(
  void * listener_data,
  DDS_DataReader * reader,
  const struct DDS_RequestedDeadlineMissedStatus * status)
{
  RMW_Connext_SubscriberStatusCondition * const self =
    reinterpret_cast<RMW_Connext_SubscriberStatusCondition *>(listener_data);

  UNUSED_ARG(reader);

  self->on_requested_deadline_missed(status);
}

void
RMW_Connext_DataReaderListener_requested_incompatible_qos(
  void * listener_data,
  DDS_DataReader * reader,
  const struct DDS_RequestedIncompatibleQosStatus * status)
{
  RMW_Connext_SubscriberStatusCondition * const self =
    reinterpret_cast<RMW_Connext_SubscriberStatusCondition *>(listener_data);

  UNUSED_ARG(reader);

  self->on_requested_incompatible_qos(status);
}

void
RMW_Connext_DataReaderListener_liveliness_changed(
  void * listener_data,
  DDS_DataReader * reader,
  const struct DDS_LivelinessChangedStatus * status)
{
  RMW_Connext_SubscriberStatusCondition * const self =
    reinterpret_cast<RMW_Connext_SubscriberStatusCondition *>(listener_data);

  UNUSED_ARG(reader);

  self->on_liveliness_changed(status);
}

void
RMW_Connext_DataReaderListener_sample_lost(
  void * listener_data,
  DDS_DataReader * reader,
  const struct DDS_SampleLostStatus * status)
{
  RMW_Connext_SubscriberStatusCondition * const self =
    reinterpret_cast<RMW_Connext_SubscriberStatusCondition *>(listener_data);

  UNUSED_ARG(reader);

  self->on_sample_lost(status);
}

void
RMW_Connext_DataReaderListener_on_data_available(
  void * listener_data,
  DDS_DataReader * reader)
{
  RMW_Connext_SubscriberStatusCondition * const self =
    reinterpret_cast<RMW_Connext_SubscriberStatusCondition *>(listener_data);

  UNUSED_ARG(reader);
  RMW_CONNEXT_LOG_DEBUG_A(
    "data available: condition=%p, reader=%p",
    (void *)self, (void *)reader);
  self->on_data();
}

void
RMW_Connext_DataWriterListener_offered_deadline_missed(
  void * listener_data,
  DDS_DataWriter * writer,
  const struct DDS_OfferedDeadlineMissedStatus * status)
{
  RMW_Connext_PublisherStatusCondition * const self =
    reinterpret_cast<RMW_Connext_PublisherStatusCondition *>(listener_data);

  UNUSED_ARG(writer);

  self->on_offered_deadline_missed(status);
}

void
RMW_Connext_DataWriterListener_offered_incompatible_qos(
  void * listener_data,
  DDS_DataWriter * writer,
  const struct DDS_OfferedIncompatibleQosStatus * status)
{
  RMW_Connext_PublisherStatusCondition * const self =
    reinterpret_cast<RMW_Connext_PublisherStatusCondition *>(listener_data);

  UNUSED_ARG(writer);

  self->on_offered_incompatible_qos(status);
}

void
RMW_Connext_DataWriterListener_liveliness_lost(
  void * listener_data,
  DDS_DataWriter * writer,
  const struct DDS_LivelinessLostStatus * status)
{
  RMW_Connext_PublisherStatusCondition * const self =
    reinterpret_cast<RMW_Connext_PublisherStatusCondition *>(listener_data);

  UNUSED_ARG(writer);

  self->on_liveliness_lost(status);
}


bool
RMW_Connext_WaitSet::on_condition_active(
  rmw_subscriptions_t * const subs,
  rmw_guard_conditions_t * const gcs,
  rmw_services_t * const srvs,
  rmw_clients_t * const cls,
  rmw_events_t * const evs)
{
  if (nullptr != subs) {
    for (size_t i = 0; i < subs->subscriber_count; ++i) {
      RMW_Connext_Subscriber * const sub =
        reinterpret_cast<RMW_Connext_Subscriber *>(subs->subscribers[i]);
      if (sub->has_data()) {
        return true;
      }
    }
  }

  if (nullptr != cls) {
    for (size_t i = 0; i < cls->client_count; ++i) {
      RMW_Connext_Client * const client =
        reinterpret_cast<RMW_Connext_Client *>(cls->clients[i]);
      if (client->subscriber()->has_data()) {
        return true;
      }
    }
  }

  if (nullptr != srvs) {
    for (size_t i = 0; i < srvs->service_count; ++i) {
      RMW_Connext_Service * const svc =
        reinterpret_cast<RMW_Connext_Service *>(srvs->services[i]);
      if (svc->subscriber()->has_data()) {
        return true;
      }
    }
  }

  if (nullptr != evs) {
    for (size_t i = 0; i < evs->event_count; ++i) {
      rmw_event_t * const event =
        reinterpret_cast<rmw_event_t *>(evs->events[i]);
      if (RMW_Connext_Event::reader_event(event)) {
        auto sub = RMW_Connext_Event::subscriber(event);
        if (sub->condition()->has_status(event->event_type)) {
          return true;
        }
      } else {
        auto pub = RMW_Connext_Event::publisher(event);
        if (pub->condition()->has_status(event->event_type)) {
          return true;
        }
      }
    }
  }

  if (nullptr != gcs) {
    for (size_t i = 0; i < gcs->guard_condition_count; ++i) {
      RMW_Connext_GuardCondition * const gcond =
        reinterpret_cast<RMW_Connext_GuardCondition *>(gcs->guard_conditions[i]);
      if (gcond->has_triggered()) {
        return true;
      }
    }
  }

  return false;
}


void
RMW_Connext_WaitSet::attach(
  rmw_subscriptions_t * const subs,
  rmw_guard_conditions_t * const gcs,
  rmw_services_t * const srvs,
  rmw_clients_t * const cls,
  rmw_events_t * const evs,
  bool & wait_active)
{
  if (nullptr != subs) {
    for (size_t i = 0; i < subs->subscriber_count; ++i) {
      RMW_Connext_Subscriber * const sub =
        reinterpret_cast<RMW_Connext_Subscriber *>(subs->subscribers[i]);
      if (sub->has_data()) {
        wait_active = true;
        return;
      }
      sub->condition()->attach(&this->condition_mutex, &this->condition);
    }
  }

  if (nullptr != cls) {
    for (size_t i = 0; i < cls->client_count; ++i) {
      RMW_Connext_Client * const client =
        reinterpret_cast<RMW_Connext_Client *>(cls->clients[i]);
      if (client->subscriber()->has_data()) {
        wait_active = true;
        return;
      }
      client->subscriber()->condition()->attach(&this->condition_mutex, &this->condition);
    }
  }

  if (nullptr != srvs) {
    for (size_t i = 0; i < srvs->service_count; ++i) {
      RMW_Connext_Service * const svc =
        reinterpret_cast<RMW_Connext_Service *>(srvs->services[i]);
      if (svc->subscriber()->has_data()) {
        wait_active = true;
        return;
      }
      svc->subscriber()->condition()->attach(&this->condition_mutex, &this->condition);
    }
  }

  if (nullptr != evs) {
    for (size_t i = 0; i < evs->event_count; ++i) {
      rmw_event_t * const event =
        reinterpret_cast<rmw_event_t *>(evs->events[i]);
      if (RMW_Connext_Event::reader_event(event)) {
        auto sub = RMW_Connext_Event::subscriber(event);
        if (sub->condition()->has_status(event->event_type)) {
          wait_active = true;
          return;
        }
        sub->condition()->attach(&this->condition_mutex, &this->condition);
      } else {
        auto pub = RMW_Connext_Event::publisher(event);
        if (pub->condition()->has_status(event->event_type)) {
          wait_active = true;
          return;
        }
        pub->condition()->attach(&this->condition_mutex, &this->condition);
      }
    }
  }

  if (nullptr != gcs) {
    for (size_t i = 0; i < gcs->guard_condition_count; ++i) {
      RMW_Connext_GuardCondition * const gcond =
        reinterpret_cast<RMW_Connext_GuardCondition *>(gcs->guard_conditions[i]);
      if (gcond->has_triggered()) {
        wait_active = true;
        return;
      }
      gcond->attach(&this->condition_mutex, &this->condition);
    }
  }
}

void
RMW_Connext_WaitSet::detach(
  rmw_subscriptions_t * const subs,
  rmw_guard_conditions_t * const gcs,
  rmw_services_t * const srvs,
  rmw_clients_t * const cls,
  rmw_events_t * const evs,
  size_t & active_conditions)
{
  if (nullptr != subs) {
    for (size_t i = 0; i < subs->subscriber_count; ++i) {
      RMW_Connext_Subscriber * const sub =
        reinterpret_cast<RMW_Connext_Subscriber *>(subs->subscribers[i]);
      sub->condition()->detach();
      if (!sub->has_data()) {
        subs->subscribers[i] = nullptr;
      } else {
        RMW_CONNEXT_LOG_DEBUG_A(
          "[wait] active subscriber: sub=%p\n",
          reinterpret_cast<void *>(sub))
        active_conditions += 1;
      }
    }
  }

  if (nullptr != cls) {
    for (size_t i = 0; i < cls->client_count; ++i) {
      RMW_Connext_Client * const client =
        reinterpret_cast<RMW_Connext_Client *>(cls->clients[i]);
      client->subscriber()->condition()->detach();
      if (!client->subscriber()->has_data()) {
        cls->clients[i] = nullptr;
      } else {
        RMW_CONNEXT_LOG_DEBUG_A(
          "[wait] active client: "
          "client=%p", (void *)client)
        active_conditions += 1;
      }
    }
  }

  if (nullptr != srvs) {
    for (size_t i = 0; i < srvs->service_count; ++i) {
      RMW_Connext_Service * const svc =
        reinterpret_cast<RMW_Connext_Service *>(srvs->services[i]);
      svc->subscriber()->condition()->detach();
      if (!svc->subscriber()->has_data()) {
        srvs->services[i] = nullptr;
      } else {
        RMW_CONNEXT_LOG_DEBUG_A(
          "[wait] active service: "
          "svc=%p", (void *)svc)
        active_conditions += 1;
      }
    }
  }

  if (nullptr != evs) {
    for (size_t i = 0; i < evs->event_count; ++i) {
      rmw_event_t * const event =
        reinterpret_cast<rmw_event_t *>(evs->events[i]);
      if (RMW_Connext_Event::reader_event(event)) {
        auto sub = RMW_Connext_Event::subscriber(event);
        sub->condition()->detach();
        if (!sub->condition()->has_status(event->event_type)) {
          evs->events[i] = nullptr;
        } else {
          RMW_CONNEXT_LOG_DEBUG_A(
            "[wait] active subscriber event: "
            "event=%p", (void *)event)
          active_conditions += 1;
        }
      } else {
        auto pub = RMW_Connext_Event::publisher(event);
        pub->condition()->detach();
        if (!pub->condition()->has_status(event->event_type)) {
          evs->events[i] = nullptr;
        } else {
          RMW_CONNEXT_LOG_DEBUG_A(
            "[wait] active publisher event: "
            "event=%p", (void *)event)
          active_conditions += 1;
        }
      }
    }
  }

  if (nullptr != gcs) {
    for (size_t i = 0; i < gcs->guard_condition_count; ++i) {
      RMW_Connext_GuardCondition * const gcond =
        reinterpret_cast<RMW_Connext_GuardCondition *>(gcs->guard_conditions[i]);
      gcond->detach();
      if (!gcond->trigger_check()) {
        gcs->guard_conditions[i] = nullptr;
      } else {
        RMW_CONNEXT_LOG_DEBUG_A(
          "[wait] active guard condition: "
          "condition=%p", (void *)gcond)
        active_conditions += 1;
      }
    }
  }
}

rmw_ret_t
RMW_Connext_WaitSet::wait(
  rmw_subscriptions_t * const subs,
  rmw_guard_conditions_t * const gcs,
  rmw_services_t * const srvs,
  rmw_clients_t * const cls,
  rmw_events_t * const evs,
  const rmw_time_t * const wait_timeout)
{
#if 1
  {
    std::lock_guard<std::mutex> lock(this->mutex_internal);
    if (this->waiting) {
      RMW_CONNEXT_LOG_ERROR_SET(
        "multiple concurrent wait()s not supported");
      return RMW_RET_ERROR;
    }
    this->waiting = true;
  }

  RMW_Connext_WaitSet * const ws = this;
  auto scope_exit = rcpputils::make_scope_exit(
    [ws]()
    {
      std::lock_guard<std::mutex> lock(ws->mutex_internal);
      ws->waiting = false;
    });
#endif

  bool already_active = false;

  this->attach(subs, gcs, srvs, cls, evs, already_active);

  bool timedout = false;

  if (!already_active) {
    RMW_CONNEXT_LOG_DEBUG_A(
      "[wait] waiting on: "
      "waitset=%p, "
      "subs=%lu, "
      "gcs=%lu, "
      "srvs=%lu, "
      "cls=%lu, "
      "evs=%lu\n",
      reinterpret_cast<void *>(this),
      (nullptr != subs) ? subs->subscriber_count : 0,
      (nullptr != gcs) ? gcs->guard_condition_count : 0,
      (nullptr != srvs) ? srvs->service_count : 0,
      (nullptr != cls) ? cls->client_count : 0,
      (nullptr != evs) ? evs->event_count : 0);

    std::unique_lock<std::mutex> lock(this->condition_mutex);

    auto on_condition_active =
      [self = this, subs, gcs, srvs, cls, evs]()
      {
        return self->on_condition_active(subs, gcs, srvs, cls, evs);
      };

    if (nullptr == wait_timeout || rmw_time_equal(*wait_timeout, RMW_DURATION_INFINITE)) {
      this->condition.wait(lock, on_condition_active);
    } else if (wait_timeout->sec > 0 || wait_timeout->nsec > 0) {
      auto n = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::seconds(wait_timeout->sec));
      n += std::chrono::nanoseconds(wait_timeout->nsec);
      timedout = !this->condition.wait_for(lock, n, on_condition_active);
    } else {
      timedout = true;
    }
  }

  size_t active_conditions = 0;
  this->detach(subs, gcs, srvs, cls, evs, active_conditions);

  RMW_CONNEXT_ASSERT(active_conditions > 0 || timedout);

  if (timedout) {
    return RMW_RET_TIMEOUT;
  }

  return RMW_RET_OK;
}

rmw_ret_t
RMW_Connext_SubscriberStatusCondition::install(
  RMW_Connext_Subscriber * const sub)
{
  DDS_DataReaderListener listener = DDS_DataReaderListener_INITIALIZER;
  DDS_StatusMask listener_mask = DDS_STATUS_MASK_NONE;

  listener.on_requested_deadline_missed =
    RMW_Connext_DataReaderListener_requested_deadline_missed;
  listener.on_requested_incompatible_qos =
    RMW_Connext_DataReaderListener_requested_incompatible_qos;
  listener.on_liveliness_changed =
    RMW_Connext_DataReaderListener_liveliness_changed;
  listener.on_sample_lost =
    RMW_Connext_DataReaderListener_sample_lost;
  listener.on_data_available =
    RMW_Connext_DataReaderListener_on_data_available;
  listener.as_listener.listener_data = this;

  listener_mask =
    DDS_REQUESTED_DEADLINE_MISSED_STATUS |
    DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS |
    DDS_LIVELINESS_CHANGED_STATUS |
    DDS_SAMPLE_LOST_STATUS |
    DDS_DATA_AVAILABLE_STATUS;

  rmw_connextdds_configure_subscriber_condition_listener(
    this, &listener, &listener_mask);

  if (DDS_RETCODE_OK !=
    DDS_DataReader_set_listener(sub->reader(), &listener, listener_mask))
  {
    RMW_CONNEXT_LOG_ERROR_SET("failed to configure reader listener")
    return RMW_RET_ERROR;
  }

  this->sub = sub;

  return RMW_RET_OK;
}

RMW_Connext_SubscriberStatusCondition::RMW_Connext_SubscriberStatusCondition(
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
  triggered_deadline(false),
  triggered_liveliness(false),
  triggered_qos(false),
  triggered_sample_lost(false),
  triggered_data(false),
  _loan_guard_condition(nullptr),
  status_deadline(DDS_RequestedDeadlineMissedStatus_INITIALIZER),
  status_qos(DDS_RequestedIncompatibleQosStatus_INITIALIZER),
  status_liveliness(DDS_LivelinessChangedStatus_INITIALIZER),
  status_sample_lost(DDS_SampleLostStatus_INITIALIZER),
  status_deadline_last(DDS_RequestedDeadlineMissedStatus_INITIALIZER),
  status_qos_last(DDS_RequestedIncompatibleQosStatus_INITIALIZER),
  status_liveliness_last(DDS_LivelinessChangedStatus_INITIALIZER),
  status_sample_lost_last(DDS_SampleLostStatus_INITIALIZER),
  sub(nullptr)
{
  if (internal) {
    this->_loan_guard_condition = DDS_GuardCondition_new();
    if (nullptr == this->_loan_guard_condition) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to allocate internal reader condition")
      throw new std::runtime_error("failed to allocate internal reader condition");
    }
  }
}

RMW_Connext_SubscriberStatusCondition::~RMW_Connext_SubscriberStatusCondition()
{
  if (nullptr != this->_loan_guard_condition) {
    if (DDS_RETCODE_OK != DDS_GuardCondition_delete(this->_loan_guard_condition)) {
      RMW_CONNEXT_LOG_ERROR_SET("failed to delete internal reader condition")
    }
  }
}

void
RMW_Connext_SubscriberStatusCondition::on_data()
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->triggered_data = true;

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }

  // Update loan guard condition's trigger value for internal endpoints
  if (nullptr != this->_loan_guard_condition) {
    (void)this->trigger_loan_guard_condition(true);
  }
}

bool
RMW_Connext_SubscriberStatusCondition::has_status(
  const rmw_event_type_t event_type)
{
  switch (event_type) {
    case RMW_EVENT_LIVELINESS_CHANGED:
      {
        return this->triggered_liveliness;
      }
    case RMW_EVENT_REQUESTED_DEADLINE_MISSED:
      {
        return this->triggered_deadline;
      }
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
      {
        return this->triggered_qos;
      }
    case RMW_EVENT_MESSAGE_LOST:
      {
        return this->triggered_sample_lost;
      }
    default:
      {
        RMW_CONNEXT_ASSERT(0)
        return false;
      }
  }
}


void
RMW_Connext_SubscriberStatusCondition::on_requested_deadline_missed(
  const DDS_RequestedDeadlineMissedStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_deadline(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_SubscriberStatusCondition::on_requested_incompatible_qos(
  const DDS_RequestedIncompatibleQosStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_qos(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_SubscriberStatusCondition::on_liveliness_changed(
  const DDS_LivelinessChangedStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_liveliness(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_SubscriberStatusCondition::on_sample_lost(
  const DDS_SampleLostStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_sample_lost(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_SubscriberStatusCondition::update_status_deadline(
  const DDS_RequestedDeadlineMissedStatus * const status)
{
  this->status_deadline = *status;
  this->triggered_deadline = true;

  this->status_deadline.total_count_change = this->status_deadline.total_count;
  this->status_deadline.total_count_change -= this->status_deadline_last.total_count;
}

void
RMW_Connext_SubscriberStatusCondition::update_status_liveliness(
  const DDS_LivelinessChangedStatus * const status)
{
  this->status_liveliness = *status;
  this->triggered_liveliness = true;

  this->status_liveliness.alive_count_change = this->status_liveliness.alive_count;
  this->status_liveliness.not_alive_count_change = this->status_liveliness.not_alive_count;
  this->status_liveliness.alive_count_change -= this->status_liveliness_last.alive_count;
  this->status_liveliness.not_alive_count_change -=
    this->status_liveliness_last.not_alive_count;
}

void
RMW_Connext_SubscriberStatusCondition::update_status_qos(
  const DDS_RequestedIncompatibleQosStatus * const status)
{
  this->status_qos = *status;
  this->triggered_qos = true;

  this->status_qos.total_count_change = this->status_qos.total_count;
  this->status_qos.total_count_change -= this->status_qos_last.total_count;
}

void
RMW_Connext_SubscriberStatusCondition::update_status_sample_lost(
  const DDS_SampleLostStatus * const status)
{
  this->status_sample_lost = *status;
  this->triggered_sample_lost = true;

  this->status_sample_lost.total_count_change = this->status_sample_lost.total_count;
  this->status_sample_lost.total_count_change -=
    this->status_sample_lost_last.total_count;
}

rmw_ret_t
RMW_Connext_PublisherStatusCondition::install(
  RMW_Connext_Publisher * const pub)
{
  DDS_DataWriterListener listener = DDS_DataWriterListener_INITIALIZER;
  DDS_StatusMask listener_mask = DDS_STATUS_MASK_NONE;

  listener.on_offered_deadline_missed =
    RMW_Connext_DataWriterListener_offered_deadline_missed;
  listener.on_offered_incompatible_qos =
    RMW_Connext_DataWriterListener_offered_incompatible_qos;
  listener.on_liveliness_lost =
    RMW_Connext_DataWriterListener_liveliness_lost;
  listener.as_listener.listener_data = this;

  listener_mask =
    DDS_OFFERED_DEADLINE_MISSED_STATUS |
    DDS_OFFERED_INCOMPATIBLE_QOS_STATUS |
    DDS_LIVELINESS_LOST_STATUS;

  if (DDS_RETCODE_OK !=
    DDS_DataWriter_set_listener(
      pub->writer(), &listener, listener_mask))
  {
    RMW_CONNEXT_LOG_ERROR_SET("failed to configure writer listener")
    return RMW_RET_ERROR;
  }

  this->pub = pub;

  return RMW_RET_OK;
}

RMW_Connext_PublisherStatusCondition::RMW_Connext_PublisherStatusCondition(
  DDS_DataWriter * const writer)
: RMW_Connext_StatusCondition(DDS_DataWriter_as_entity(writer)),
  triggered_deadline(false),
  triggered_liveliness(false),
  triggered_qos(false),
  status_deadline(DDS_OfferedDeadlineMissedStatus_INITIALIZER),
  status_qos(DDS_OfferedIncompatibleQosStatus_INITIALIZER),
  status_liveliness(DDS_LivelinessLostStatus_INITIALIZER),
  status_deadline_last(DDS_OfferedDeadlineMissedStatus_INITIALIZER),
  status_qos_last(DDS_OfferedIncompatibleQosStatus_INITIALIZER),
  status_liveliness_last(DDS_LivelinessLostStatus_INITIALIZER)
{}

bool
RMW_Connext_PublisherStatusCondition::has_status(
  const rmw_event_type_t event_type)
{
  switch (event_type) {
    case RMW_EVENT_LIVELINESS_LOST:
      {
        return this->triggered_liveliness;
      }
    case RMW_EVENT_OFFERED_DEADLINE_MISSED:
      {
        return this->triggered_deadline;
      }
    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
      {
        return this->triggered_qos;
      }
    default:
      RMW_CONNEXT_ASSERT(0)
      return false;
  }
}


void
RMW_Connext_PublisherStatusCondition::on_offered_deadline_missed(
  const DDS_OfferedDeadlineMissedStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_deadline(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_PublisherStatusCondition::on_offered_incompatible_qos(
  const DDS_OfferedIncompatibleQosStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_qos(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_PublisherStatusCondition::on_liveliness_lost(
  const DDS_LivelinessLostStatus * const status)
{
  std::lock_guard<std::mutex> lock(this->mutex_internal);

  this->update_status_liveliness(status);

  if (nullptr != this->waitset_condition) {
    this->waitset_condition->notify_one();
  }
}

void
RMW_Connext_PublisherStatusCondition::update_status_deadline(
  const DDS_OfferedDeadlineMissedStatus * const status)
{
  this->status_deadline = *status;
  this->triggered_deadline = true;

  this->status_deadline.total_count_change = this->status_deadline.total_count;
  this->status_deadline.total_count_change -= this->status_deadline_last.total_count;
}

void
RMW_Connext_PublisherStatusCondition::update_status_liveliness(
  const DDS_LivelinessLostStatus * const status)
{
  this->status_liveliness = *status;
  this->triggered_liveliness = true;

  this->status_liveliness.total_count_change = this->status_liveliness.total_count;
  this->status_liveliness.total_count_change -= this->status_liveliness_last.total_count;
}

void
RMW_Connext_PublisherStatusCondition::update_status_qos(
  const DDS_OfferedIncompatibleQosStatus * const status)
{
  this->status_qos = *status;
  this->triggered_qos = true;

  this->status_qos.total_count_change = this->status_qos.total_count;
  this->status_qos.total_count_change -= this->status_qos_last.total_count;
}
#endif /* !RMW_CONNEXT_CPP_STD_WAITSETS */
