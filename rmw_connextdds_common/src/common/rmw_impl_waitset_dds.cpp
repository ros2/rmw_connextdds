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

#include <vector>

#include "rmw_connextdds/rmw_impl.hpp"

#if RMW_CONNEXT_HAVE_TIME_UTILS
#include "rmw_dds_common/time_utils.hpp"
#endif /* RMW_CONNEXT_HAVE_TIME_UTILS */

#if !RMW_CONNEXT_CPP_STD_WAITSETS
/******************************************************************************
 * WaitSet
 ******************************************************************************/

template<typename T>
bool
RMW_Connext_WaitSet::require_attach(
  const std::vector<T *> & attached_els,
  const size_t new_els_count,
  void ** const new_els)
{
  if (nullptr == new_els || 0 == new_els_count) {
    return attached_els.size() > 0;
  } else if (new_els_count != attached_els.size()) {
    return true;
  } else {
    const void * const attached_data =
      static_cast<const void *>(attached_els.data());
    void * const new_els_data = static_cast<void *>(new_els);

    const size_t cmp_size = new_els_count * sizeof(void *);

    return memcmp(attached_data, new_els_data, cmp_size) != 0;
  }
}

rmw_ret_t
RMW_Connext_WaitSet::detach()
{
  bool failed = false;

  for (auto && sub : this->attached_subscribers) {
    RMW_Connext_StatusCondition * const cond = sub->condition();
    {
      std::lock_guard<std::mutex> lock(cond->mutex_internal);
      rmw_ret_t rc = cond->detach();
      if (RMW_RET_OK != rc) {
        RMW_CONNEXT_LOG_ERROR("failed to detach subscriber's condition")
        failed = true;
      }
    }
  }
  this->attached_subscribers.clear();

  for (auto && gc : this->attached_conditions) {
    {
      std::lock_guard<std::mutex> lock(gc->mutex_internal);
      rmw_ret_t rc = gc->detach();
      if (RMW_RET_OK != rc) {
        RMW_CONNEXT_LOG_ERROR("failed to detach guard condition")
        failed = true;
      }
    }
  }
  this->attached_conditions.clear();

  for (auto && client : this->attached_clients) {
    RMW_Connext_StatusCondition * const cond = client->subscriber()->condition();
    {
      std::lock_guard<std::mutex> lock(cond->mutex_internal);
      rmw_ret_t rc = cond->detach();
      if (RMW_RET_OK != rc) {
        RMW_CONNEXT_LOG_ERROR("failed to detach client's condition")
        failed = true;
      }
    }
  }
  this->attached_clients.clear();

  for (auto && service : this->attached_services) {
    RMW_Connext_StatusCondition * const cond = service->subscriber()->condition();
    {
      std::lock_guard<std::mutex> lock(cond->mutex_internal);
      rmw_ret_t rc = cond->detach();
      if (RMW_RET_OK != rc) {
        RMW_CONNEXT_LOG_ERROR("failed to detach service's condition")
        failed = true;
      }
    }
  }
  this->attached_services.clear();

  for (auto && e : this->attached_events) {
    auto e_cached = this->attached_events_cache[e];
    RMW_Connext_StatusCondition * const cond = RMW_Connext_Event::condition(&e_cached);
    {
      std::lock_guard<std::mutex> lock(cond->mutex_internal);
      rmw_ret_t rc = cond->detach();
      if (RMW_RET_OK != rc) {
        RMW_CONNEXT_LOG_ERROR("failed to detach event's condition")
        failed = true;
      }
    }
  }
  this->attached_events.clear();
  this->attached_events_cache.clear();

  return failed ? RMW_RET_ERROR : RMW_RET_OK;
}


rmw_ret_t
RMW_Connext_WaitSet::attach(
  rmw_subscriptions_t * const subs,
  rmw_guard_conditions_t * const gcs,
  rmw_services_t * const srvs,
  rmw_clients_t * const cls,
  rmw_events_t * const evs)
{
  const bool refresh_attach_subs =
    this->require_attach(
    this->attached_subscribers,
    (nullptr != subs) ? subs->subscriber_count : 0,
    (nullptr != subs) ? subs->subscribers : nullptr),
    refresh_attach_gcs =
    this->require_attach(
    this->attached_conditions,
    (nullptr != gcs) ? gcs->guard_condition_count : 0,
    (nullptr != gcs) ? gcs->guard_conditions : nullptr),
    refresh_attach_srvs =
    this->require_attach(
    this->attached_services,
    (nullptr != srvs) ? srvs->service_count : 0,
    (nullptr != srvs) ? srvs->services : nullptr),
    refresh_attach_cls =
    this->require_attach(
    this->attached_clients,
    (nullptr != cls) ? cls->client_count : 0,
    (nullptr != cls) ? cls->clients : nullptr),
    refresh_attach_evs =
    this->require_attach(
    this->attached_events,
    (nullptr != evs) ? evs->event_count : 0,
    (nullptr != evs) ? evs->events : nullptr);
  const bool refresh_attach =
    refresh_attach_subs ||
    refresh_attach_gcs ||
    refresh_attach_evs ||
    refresh_attach_srvs ||
    refresh_attach_cls;

  if (!refresh_attach) {
    // nothing to do since lists of attached elements didn't change
    return RMW_RET_OK;
  }

  rmw_ret_t rc = this->detach();
  if (RMW_RET_OK != rc) {
    RMW_CONNEXT_LOG_ERROR("failed to detach conditions from waitset")
    return rc;
  }

  // First iterate over events, and reset the "enabled statuses" of the
  // target entity's status condition. We could skip any subscriber that is
  // also passed in for data (since the "enabled statuses" will be reset to
  // DATA_AVAILABLE only for these subscribers), but we reset them anyway to
  // avoid having to search the subscriber's list for each one.
  if (nullptr != evs) {
    for (size_t i = 0; i < evs->event_count; ++i) {
      rmw_event_t * const event =
        reinterpret_cast<rmw_event_t *>(evs->events[i]);
      RMW_Connext_StatusCondition * const cond =
        RMW_Connext_Event::condition(event);
      RMW_Connext_WaitSet * const otherws = cond->attached_waitset;
      bool detached = false;
      if (nullptr != otherws && this != otherws) {
        otherws->invalidate(cond);
        detached = true;
      }
      {
        std::lock_guard<std::mutex> lock(cond->mutex_internal);
        if (cond->deleted) {
          return RMW_RET_ERROR;
        }
        if (detached) {
          cond->attached_waitset = nullptr;
        }
        rmw_ret_t rc = cond->reset_statuses();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to reset event's condition")
          return rc;
        }
      }
    }
  }

  if (nullptr != subs) {
    for (size_t i = 0; i < subs->subscriber_count; ++i) {
      RMW_Connext_Subscriber * const sub =
        reinterpret_cast<RMW_Connext_Subscriber *>(subs->subscribers[i]);
      RMW_Connext_SubscriberStatusCondition * const cond = sub->condition();
      RMW_Connext_WaitSet * const otherws = cond->attached_waitset;
      bool detached = false;
      if (nullptr != otherws && this != otherws) {
        otherws->invalidate(cond);
        detached = true;
      }
      {
        std::lock_guard<std::mutex> lock(cond->mutex_internal);
        if (cond->deleted) {
          return RMW_RET_ERROR;
        }
        if (detached) {
          cond->attached_waitset = nullptr;
        }
        rmw_ret_t rc = cond->reset_statuses();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to reset subscriber's condition")
          return rc;
        }
        rc = cond->enable_statuses(DDS_DATA_AVAILABLE_STATUS);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to enable subscriber's condition")
          return rc;
        }
        rc = cond->attach(this);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach subscriber's condition")
          return rc;
        }
        rc = cond->attach_data();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach subscriber's data condition")
          return rc;
        }
      }
      this->attached_subscribers.push_back(sub);
    }
  }

  if (nullptr != cls) {
    for (size_t i = 0; i < cls->client_count; ++i) {
      RMW_Connext_Client * const client =
        reinterpret_cast<RMW_Connext_Client *>(cls->clients[i]);
      RMW_Connext_SubscriberStatusCondition * const cond = client->subscriber()->condition();
      RMW_Connext_WaitSet * const otherws = cond->attached_waitset;
      bool detached = false;
      if (nullptr != otherws && this != otherws) {
        otherws->invalidate(cond);
        detached = true;
      }
      {
        std::lock_guard<std::mutex> lock(cond->mutex_internal);
        if (cond->deleted) {
          return RMW_RET_ERROR;
        }
        if (detached) {
          cond->attached_waitset = nullptr;
        }
        rmw_ret_t rc = cond->reset_statuses();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to reset subscriber's condition")
          return rc;
        }
        rc = cond->enable_statuses(DDS_DATA_AVAILABLE_STATUS);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to enable client's condition")
          return rc;
        }
        rc = cond->attach(this);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach client's condition")
          return rc;
        }
        rc = cond->attach_data();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach client's data condition")
          return rc;
        }
      }
      this->attached_clients.push_back(client);
    }
  }

  if (nullptr != srvs) {
    for (size_t i = 0; i < srvs->service_count; ++i) {
      RMW_Connext_Service * const svc =
        reinterpret_cast<RMW_Connext_Service *>(srvs->services[i]);
      RMW_Connext_SubscriberStatusCondition * const cond = svc->subscriber()->condition();
      RMW_Connext_WaitSet * const otherws = cond->attached_waitset;
      bool detached = false;
      if (nullptr != otherws && this != otherws) {
        otherws->invalidate(cond);
        detached = true;
      }
      {
        std::lock_guard<std::mutex> lock(cond->mutex_internal);
        if (cond->deleted) {
          return RMW_RET_ERROR;
        }
        if (detached) {
          cond->attached_waitset = nullptr;
        }
        rmw_ret_t rc = cond->reset_statuses();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to reset subscriber's condition")
          return rc;
        }
        rc = cond->enable_statuses(DDS_DATA_AVAILABLE_STATUS);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to enable service's condition")
          return rc;
        }
        rc = cond->attach(this);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach service's condition")
          return rc;
        }
        rc = cond->attach_data();
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach service's data condition")
          return rc;
        }
      }
      this->attached_services.push_back(svc);
    }
  }

  if (nullptr != evs) {
    for (size_t i = 0; i < evs->event_count; ++i) {
      rmw_event_t * const event =
        reinterpret_cast<rmw_event_t *>(evs->events[i]);
      RMW_Connext_StatusCondition * const cond =
        RMW_Connext_Event::condition(event);
      {
        std::lock_guard<std::mutex> lock(cond->mutex_internal);
        if (cond->deleted) {
          return RMW_RET_ERROR;
        }
        const DDS_StatusKind evt = ros_event_to_dds(event->event_type, nullptr);
        rmw_ret_t rc = cond->enable_statuses(evt);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to enable event's condition")
          return rc;
        }
        rc = cond->attach(this);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach event's condition")
          return rc;
        }
      }
      this->attached_events.push_back(event);
      // Cache a shallow copy of the rmw_event_t structure so that we may
      // access it safely during detach(), even if the original event has been
      // deleted already by that time.
      this->attached_events_cache.emplace(event, *event);
    }
  }

  if (nullptr != gcs) {
    for (size_t i = 0; i < gcs->guard_condition_count; ++i) {
      RMW_Connext_GuardCondition * const gcond =
        reinterpret_cast<RMW_Connext_GuardCondition *>(gcs->guard_conditions[i]);
      RMW_Connext_WaitSet * const otherws = gcond->attached_waitset;
      bool detached = false;
      if (nullptr != otherws && this != otherws) {
        otherws->invalidate(gcond);
        detached = true;
      }
      {
        std::lock_guard<std::mutex> lock(gcond->mutex_internal);
        if (gcond->deleted) {
          return RMW_RET_ERROR;
        }
        if (detached) {
          gcond->attached_waitset = nullptr;
        }
        rmw_ret_t rc = gcond->attach(this);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR("failed to attach guard condition")
          return rc;
        }
      }
      this->attached_conditions.push_back(gcond);
    }
  }
  return RMW_RET_OK;
}

bool
RMW_Connext_WaitSet::active_condition(RMW_Connext_Condition * const cond)
{
  DDS_Long active_len = DDS_ConditionSeq_get_length(&this->active_conditions);
  bool active = false;

  for (DDS_Long i = 0; i < active_len && !active; i++) {
    DDS_Condition * const acond =
      *DDS_ConditionSeq_get_reference(&this->active_conditions, i);
    active = cond->owns(acond);
  }

  return active;
}

bool
RMW_Connext_WaitSet::is_attached(RMW_Connext_Condition * const cond)
{
  for (auto && sub : this->attached_subscribers) {
    if (sub->condition() == cond) {
      return true;
    }
  }

  for (auto && gc : this->attached_conditions) {
    if (gc == cond) {
      return true;
    }
  }

  for (auto && client : this->attached_clients) {
    if (client->subscriber()->condition() == cond) {
      return true;
    }
  }

  for (auto && service : this->attached_services) {
    if (service->subscriber()->condition() == cond) {
      return true;
    }
  }

  for (auto && e : this->attached_events) {
    auto e_cached = this->attached_events_cache[e];
    if (RMW_Connext_Event::condition(&e_cached) == cond) {
      return true;
    }
  }

  return false;
}

rmw_ret_t
RMW_Connext_WaitSet::process_wait(
  rmw_subscriptions_t * const subs,
  rmw_guard_conditions_t * const gcs,
  rmw_services_t * const srvs,
  rmw_clients_t * const cls,
  rmw_events_t * const evs,
  size_t & active_conditions)
{
  bool failed = false;
  size_t i = 0;

  // If any of the attached conditions has become "invalid" while we were
  // waiting, we will finish processing the results, and detach all existing
  // conditions at the end, to make sure that no stale references is stored by
  // the waitset after returning from the wait() call.
  bool valid = true;

  i = 0;
  for (auto && sub : this->attached_subscribers) {
    // Check if the subscriber has some data already cached from the DataReader,
    // or check the DataReader's cache and loan samples if needed. If empty,
    // remove subscriber from returned list.
    if (!sub->has_data()) {
      subs->subscribers[i] = nullptr;
    } else {
      active_conditions += 1;
    }
    i += 1;
    valid = valid && !sub->condition()->deleted;
  }

  i = 0;
  for (auto && gc : this->attached_conditions) {
    // Scan active conditions returned by wait() looking for this guard condition
    if (!this->active_condition(gc)) {
      gcs->guard_conditions[i] = nullptr;
    } else {
      // reset conditions's trigger value. There is a risk of "race condition"
      // here since resetting the trigger value might overwrite a positive
      // trigger set by the upstream. In general, the DDS API expects guard
      // conditions to be fully managed by the application, exactly to avoid
      // this type of (pretty much unsolvable) issue (hence why
      // DDS_WaitSet_wait() will not automatically reset the trigger value of
      // an attached guard conditions).
      if (RMW_RET_OK != gc->reset_trigger()) {
        failed = true;
      }
      active_conditions += 1;
    }
    i += 1;
    valid = valid && !gc->deleted;
  }

  i = 0;
  for (auto && client : this->attached_clients) {
    if (!client->subscriber()->has_data()) {
      cls->clients[i] = nullptr;
    } else {
      active_conditions += 1;
    }
    i += 1;
    valid = valid && !client->subscriber()->condition()->deleted;
  }

  i = 0;
  for (auto && service : this->attached_services) {
    if (!service->subscriber()->has_data()) {
      srvs->services[i] = nullptr;
    } else {
      active_conditions += 1;
    }
    i += 1;
  }

  i = 0;
  for (auto && e : this->attached_events) {
    auto e_cached = this->attached_events_cache[e];
    // Check if associated DDS status is active on the associated entity
    if (!RMW_Connext_Event::active(&e_cached)) {
      evs->events[i] = nullptr;
    } else {
      active_conditions += 1;
    }
    i += 1;
    valid = valid && !RMW_Connext_Event::condition(&e_cached)->deleted;
  }

  if (!failed && valid) {
    return RMW_RET_OK;
  }

  return RMW_RET_ERROR;
}

static
rmw_ret_t
rmw_connextdds_duration_from_ros_time(
  DDS_Duration_t * const duration,
  const rmw_time_t * const ros_time)
{
#if RMW_CONNEXT_HAVE_TIME_UTILS
  rmw_time_t in_time = rmw_dds_common::clamp_rmw_time_to_dds_time(*ros_time);
#else
  // TODO(asorbini) In older versions, this function ignores possible overflows
  // which occur if (ros_time->sec > INT32_MAX || ros_time->nsec > UINT32_MAX)
  rmw_time_t in_time = *ros_time;
#endif /* RMW_CONNEXT_HAVE_TIME_UTILS */

  duration->sec = static_cast<DDS_Long>(in_time.sec);
  duration->nanosec = static_cast<DDS_UnsignedLong>(in_time.nsec);
  return RMW_RET_OK;
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
  {
    std::unique_lock<std::mutex> lock(this->mutex_internal);
    bool already_taken = false;
    switch (this->state) {
      case RMW_CONNEXT_WAITSET_FREE:
        {
          // waitset is available
          break;
        }
      case RMW_CONNEXT_WAITSET_INVALIDATING:
        {
          // waitset is currently being invalidated, wait for other thread to
          // complete.
          this->state_cond.wait(lock);
          already_taken = (RMW_CONNEXT_WAITSET_FREE != this->state);
          break;
        }
      default:
        {
          already_taken = true;
          break;
        }
    }
    if (already_taken) {
      // waitset is owned by another thread
      RMW_CONNEXT_LOG_ERROR_SET(
        "multiple concurrent wait()s not supported");
      return RMW_RET_ERROR;
    }
    this->state = RMW_CONNEXT_WAITSET_ACQUIRING;
  }
  // Notify condition variable of state transition
  this->state_cond.notify_all();

  RMW_Connext_WaitSet * const ws = this;
  // If we return with an error, then try to detach all conditions to leave
  // the waitset in a "clean" state.
  auto scope_exit_detach = rcpputils::make_scope_exit(
    [ws]()
    {
      if (RMW_RET_OK != ws->detach()) {
        RMW_CONNEXT_LOG_ERROR("failed to detach conditions from waitset")
      }
    });

  // After handling a possible error condition (i.e. clearing the waitset),
  // transition back to "FREE" state.
  auto scope_exit = rcpputils::make_scope_exit(
    [ws]()
    {
      // transition waitset back to FREE state on exit
      {
        std::lock_guard<std::mutex> lock(ws->mutex_internal);
        ws->state = RMW_CONNEXT_WAITSET_FREE;
      }
      // Notify condition variable of state transition
      ws->state_cond.notify_all();
    });

  rmw_ret_t rc = this->attach(subs, gcs, srvs, cls, evs);
  if (RMW_RET_OK != rc) {
    return rc;
  }

  const size_t attached_count =
    this->attached_subscribers.size() +
    this->attached_conditions.size() +
    this->attached_clients.size() +
    this->attached_services.size() +
    this->attached_events.size();

  if (attached_count > INT32_MAX) {
    RMW_CONNEXT_LOG_ERROR("too many conditions attached to waitset")
    return RMW_RET_ERROR;
  }

  if (!DDS_ConditionSeq_ensure_length(
      &this->active_conditions,
      static_cast<DDS_Long>(attached_count),
      static_cast<DDS_Long>(attached_count)))
  {
    RMW_CONNEXT_LOG_ERROR("failed to resize conditions sequence")
    return RMW_RET_ERROR;
  }

  DDS_Duration_t wait_duration = DDS_DURATION_INFINITE;
  if (nullptr != wait_timeout) {
    rc = rmw_connextdds_duration_from_ros_time(&wait_duration, wait_timeout);
    if (RMW_RET_OK != rc) {
      return rc;
    }
  }

  // transition to state BLOCKED
  {
    std::lock_guard<std::mutex> lock(this->mutex_internal);
    this->state = RMW_CONNEXT_WAITSET_BLOCKED;
  }
  // Notify condition variable of state transition
  this->state_cond.notify_all();

  const DDS_ReturnCode_t wait_rc =
    DDS_WaitSet_wait(this->waitset, &this->active_conditions, &wait_duration);

  if (DDS_RETCODE_OK != wait_rc && DDS_RETCODE_TIMEOUT != wait_rc) {
    RMW_CONNEXT_LOG_ERROR_A_SET("DDS wait failed: %d", wait_rc)
    return RMW_RET_ERROR;
  }

  // transition to state RELEASING
  {
    std::lock_guard<std::mutex> lock(this->mutex_internal);
    this->state = RMW_CONNEXT_WAITSET_RELEASING;
  }
  // Notify condition variable of state transition
  this->state_cond.notify_all();

  size_t active_conditions = 0;
  rc = this->process_wait(subs, gcs, srvs, cls, evs, active_conditions);
  if (RMW_RET_OK != rc) {
    RMW_CONNEXT_LOG_ERROR("failed to process wait result")
    return rc;
  }

  scope_exit_detach.cancel();

  RMW_CONNEXT_ASSERT(active_conditions > 0 || DDS_RETCODE_TIMEOUT == wait_rc)

  if (DDS_RETCODE_TIMEOUT == wait_rc) {
    // TODO(asorbini) This error was added to have a unit test that
    // generates random errors, and expects the RMW to call RMW_SET_ERROR_MSG()
    // when it returns a retcode other than "OK". Adding this log is quite
    // intrusive, since `rcl` will print out an error on every wait time-out,
    // which is really not an error, but a common occurrance in any ROS/DDS
    // application. For this reason, the code is disabled, but kept in
    // to keep track of the potential issue with that unit test (which should
    // be resolved by making it accept "timeout" as a valid retcode without
    // an error message).
#if 0
    rmw_reset_error();
    RMW_SET_ERROR_MSG("DDS wait timed out");
#endif
    return RMW_RET_TIMEOUT;
  }

  return RMW_RET_OK;
}


rmw_ret_t
RMW_Connext_WaitSet::invalidate(RMW_Connext_Condition * const condition)
{
  std::unique_lock<std::mutex> lock(this->mutex_internal);

  // Scan attached elements to see if condition is still attached.
  // If the invalidated condition is not attached, then there's nothing to do,
  // since the waitset is already free from potential stale references.
  if (!this->is_attached(condition)) {
    return RMW_RET_OK;
  }

  // If the waitset is "FREE" then we can just mark it as "INVALIDATING",
  // do the clean up, and release it. A wait()'ing thread will detect the
  // "INVALIDATING" state and block until notified.
  if (this->state == RMW_CONNEXT_WAITSET_FREE) {
    this->state = RMW_CONNEXT_WAITSET_INVALIDATING;
    lock.unlock();

    rmw_ret_t rc = this->detach();
    if (RMW_RET_OK != rc) {
      RMW_CONNEXT_LOG_ERROR("failed to detach conditions on invalidate")
    }

    lock.lock();
    this->state = RMW_CONNEXT_WAITSET_FREE;
    lock.unlock();
    this->state_cond.notify_all();
    return rc;
  }

  // Waitset is currently inside a wait() call. If the state is not "ACQUIRING"
  // then it means the user is trying to delete a condition while simultaneously
  // waiting on it. This is an error.
  if (this->state != RMW_CONNEXT_WAITSET_ACQUIRING) {
    RMW_CONNEXT_LOG_ERROR_SET("cannot delete and wait on the same object")
    return RMW_RET_ERROR;
  }

  // Block on state_cond and wait for the next state transition, at which
  // point the condition must have been detached, or we can return an error.
  this->state_cond.wait(lock);

  if (this->is_attached(condition)) {
    RMW_CONNEXT_LOG_ERROR_SET("deleted condition not detached")
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
RMW_Connext_SubscriberStatusCondition::install()
{
  DDS_DataReaderListener listener = DDS_DataReaderListener_INITIALIZER;
  DDS_StatusMask listener_mask = DDS_STATUS_MASK_NONE;

  listener.as_listener.listener_data = this;

  rmw_connextdds_configure_subscriber_condition_listener(
    this, &listener, &listener_mask);

  // TODO(asorbini) only call set_listener() if actually setting something?
  if (DDS_STATUS_MASK_NONE != listener_mask) {
    if (DDS_RETCODE_OK !=
      DDS_DataReader_set_listener(this->reader, &listener, listener_mask))
    {
      RMW_CONNEXT_LOG_ERROR_SET("failed to configure reader listener")
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}

/******************************************************************************
 * Event wrapper
 ******************************************************************************/
RMW_Connext_StatusCondition *
RMW_Connext_Event::condition(const rmw_event_t * const event)
{
  if (RMW_Connext_Event::reader_event(event)) {
    return RMW_Connext_Event::subscriber(event)->condition();
  } else {
    return RMW_Connext_Event::publisher(event)->condition();
  }
}

bool
RMW_Connext_Event::active(rmw_event_t * const event)
{
  RMW_Connext_StatusCondition * condition = nullptr;
  if (RMW_Connext_Event::reader_event(event)) {
    condition = RMW_Connext_Event::subscriber(event)->condition();
  } else {
    condition = RMW_Connext_Event::publisher(event)->condition();
  }
  return condition->has_status(event->event_type);
}

#endif /* !RMW_CONNEXT_CPP_STD_WAITSETS */
