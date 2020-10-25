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

#ifndef RMW_CONNEXT__RMW_IMPL_HPP_
#define RMW_CONNEXT__RMW_IMPL_HPP_

#include "rmw_connextdds_cpp/context.hpp"
#include "rmw_connextdds_cpp/type_support.hpp"

#include "rmw_connextdds_cpp/demangle.hpp"
#include "rmw_connextdds_cpp/namespace_prefix.hpp"

#include "rcutils/types/uint8_array.h"
#include "rmw/event.h"

rcutils_ret_t
rcutils_uint8_array_copy(
    rcutils_uint8_array_t *const dst,
    const rcutils_uint8_array_t *const src);

#if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY
rmw_qos_policy_kind_t
dds_qos_policy_to_rmw_qos_policy(const DDS_QosPolicyId_t last_policy_id);
#endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY */


/******************************************************************************
 * StdWaitSet: custom waitset implementation using std library
 ******************************************************************************/

class RMW_Connext_Publisher;
class RMW_Connext_Subscriber;

#include <condition_variable>
#include <mutex>
#include <atomic>

#include "rcpputils/thread_safety_annotations.hpp"

class RMW_Connext_StdWaitSet
{
public:

    RMW_Connext_StdWaitSet()
    : waiting(false)
    {

    }

    rmw_ret_t
    wait(
        rmw_subscriptions_t *const subs,
        rmw_guard_conditions_t *const gcs,
        rmw_services_t *const srvs,
        rmw_clients_t *const cls,
        rmw_events_t *const evs,
        const rmw_time_t *const wait_timeout);

protected:

    void
    attach(
        rmw_subscriptions_t *const subs,
        rmw_guard_conditions_t *const gcs,
        rmw_services_t *const srvs,
        rmw_clients_t *const cls,
        rmw_events_t *const evs,
        bool &already_active);
    
    void
    detach(
        rmw_subscriptions_t *const subs,
        rmw_guard_conditions_t *const gcs,
        rmw_services_t *const srvs,
        rmw_clients_t *const cls,
        rmw_events_t *const evs,
        size_t &active_conditions);

    bool
    on_condition_active(
        rmw_subscriptions_t *const subs,
        rmw_guard_conditions_t *const gcs,
        rmw_services_t *const srvs,
        rmw_clients_t *const cls,
        rmw_events_t *const evs);

    std::mutex mutex_internal;
    bool waiting;
    std::condition_variable condition;
    std::mutex condition_mutex;
};

class RMW_Connext_StdCondition
{
public:
    RMW_Connext_StdCondition()
    : mutex_internal(),
      waitset_mutex(nullptr),
      waitset_condition(nullptr)
    {

    }

    void
    attach(
        std::mutex *const waitset_mutex,
        std::condition_variable *const waitset_condition)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);
        this->waitset_mutex = waitset_mutex;
        this->waitset_condition = waitset_condition;
    }

    void
    detach()
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);
        this->waitset_mutex = nullptr;
        this->waitset_condition = nullptr;
    }

protected:
    std::mutex mutex_internal;
    std::mutex * waitset_mutex RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    std::condition_variable * waitset_condition RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
};


class RMW_Connext_StdGuardCondition : public RMW_Connext_StdCondition
{
public:

    RMW_Connext_StdGuardCondition(const bool internal = false)
    : trigger_value(false),
      internal(internal),
      gcond(nullptr)
    {
        if (this->internal)
        {
            this->gcond = DDS_GuardCondition_new();
            if (nullptr == this->gcond)
            {
                RMW_CONNEXT_LOG_ERROR("failed to allocate dds guard condition")
            }
        }
    }

    ~RMW_Connext_StdGuardCondition()
    {
        if (nullptr != this->gcond)
        {
            DDS_GuardCondition_delete(this->gcond);
        }
    }

    DDS_GuardCondition*
    guard_condition() const
    {
        return this->gcond;
    }

    bool
    trigger_check()
    {
        return this->trigger_value.exchange(false);
    }

    bool
    has_triggered()
    {
        return this->trigger_value;
    }

    void
    trigger()
    {
        if (internal)
        {
            if (DDS_RETCODE_OK !=
                    DDS_GuardCondition_set_trigger_value(
                        this->gcond, DDS_BOOLEAN_TRUE))
            {
                RMW_CONNEXT_LOG_ERROR("failed to trigger guard condition")
                return;
            }
            
            return;
        }

        std::lock_guard<std::mutex> lock(this->mutex_internal);

        if (this->waitset_mutex != nullptr)
        {
            std::unique_lock<std::mutex> clock(*this->waitset_mutex);
            this->trigger_value = true;
            clock.unlock();
            this->waitset_condition->notify_one();
        }
        else
        {
            this->trigger_value = true;
        }
    }

protected:
    std::atomic_bool trigger_value;
    bool internal;
    DDS_GuardCondition *gcond;
};

class RMW_Connext_StdStatusCondition : public RMW_Connext_StdCondition
{

};

void
RMW_Connext_DataWriterListener_offered_deadline_missed(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_OfferedDeadlineMissedStatus *status);

void
RMW_Connext_DataWriterListener_offered_incompatible_qos(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_OfferedIncompatibleQosStatus *status);

void
RMW_Connext_DataWriterListener_liveliness_lost(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_LivelinessLostStatus *status);

class RMW_Connext_StdPublisherStatusCondition : public RMW_Connext_StdStatusCondition
{
public:

    RMW_Connext_StdPublisherStatusCondition()
    : triggered_deadline(false),
      triggered_liveliness(false),
      triggered_qos(false)
    {
        const DDS_OfferedDeadlineMissedStatus def_status_deadline =
            DDS_OfferedDeadlineMissedStatus_INITIALIZER;
        const DDS_OfferedIncompatibleQosStatus def_status_qos =
            DDS_OfferedIncompatibleQosStatus_INITIALIZER;
        const DDS_LivelinessLostStatus def_status_liveliness = 
            DDS_LivelinessLostStatus_INITIALIZER;
        
        this->status_deadline = def_status_deadline;
        this->status_liveliness = def_status_liveliness;
        this->status_qos = def_status_qos;
    }

    bool
    has_status(const rmw_event_type_t event_type)
    {
        switch (event_type)
        {
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
            /* TODO assert unreachable */
            return false;
        }
    }

    bool
    get_status(const rmw_event_type_t event_type, void *const event_info)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        switch (event_type)
        {
        case RMW_EVENT_LIVELINESS_LOST:
        {
            rmw_liveliness_lost_status_t *status =
                (rmw_liveliness_lost_status_t*)event_info;
            
            status->total_count = this->status_liveliness.total_count;
            status->total_count_change =
                this->status_liveliness.total_count_change;

            this->status_liveliness.total_count_change = 0;
            this->triggered_liveliness = false;
            break;
        }
        case RMW_EVENT_OFFERED_DEADLINE_MISSED:
        {
            rmw_offered_deadline_missed_status_t * status =
                (rmw_offered_deadline_missed_status_t *)event_info;
            
            status->total_count = this->status_deadline.total_count;
            status->total_count_change =
                this->status_deadline.total_count_change;
            
            this->status_deadline.total_count_change = 0;
            this->triggered_deadline = false;
            break;
        }
        #if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY
        case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
        {
            rmw_offered_qos_incompatible_event_status_t *const status =
                (rmw_offered_qos_incompatible_event_status_t*)event_info;
            
            status->total_count = this->status_qos.total_count;
            status->total_count_change =
                this->status_qos.total_count_change;
            status->last_policy_kind =
                dds_qos_policy_to_rmw_qos_policy(
                    this->status_qos.last_policy_id);
            
            this->status_qos.total_count_change = 0;
            this->triggered_qos = false;
            break;
        }
        #endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY */
        default:
            /* TODO assert unreachable */
            return false;
        }

        return true;
    }

    friend
    void
    RMW_Connext_DataWriterListener_offered_deadline_missed(
        void *listener_data,
        DDS_DataWriter *writer,
        const struct DDS_OfferedDeadlineMissedStatus *status);
    
    friend
    void
    RMW_Connext_DataWriterListener_offered_incompatible_qos(
        void *listener_data,
        DDS_DataWriter *writer,
        const struct DDS_OfferedIncompatibleQosStatus *status);
    
    friend
    void
    RMW_Connext_DataWriterListener_liveliness_lost(
        void *listener_data,
        DDS_DataWriter *writer,
        const struct DDS_LivelinessLostStatus *status);
    
    rmw_ret_t
    install(RMW_Connext_Publisher *const pub);

    void
    on_offered_deadline_missed(
        const DDS_OfferedDeadlineMissedStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_deadline(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

    void
    on_offered_incompatible_qos(
        const DDS_OfferedIncompatibleQosStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_qos(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

    void
    on_liveliness_lost(
        const DDS_LivelinessLostStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_liveliness(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

protected:
    void update_status_deadline(
            const DDS_OfferedDeadlineMissedStatus *const status)
    {
        this->status_deadline = *status;
        this->triggered_deadline = true;
    }

    void update_status_liveliness(
            const DDS_LivelinessLostStatus *const status)
    {
        this->status_liveliness = *status;
        this->triggered_liveliness = true;
    }

    void update_status_qos(
            const DDS_OfferedIncompatibleQosStatus *const status)
    {
        this->status_qos = *status;
        this->triggered_qos = true;
    }

    bool triggered_deadline;
    bool triggered_liveliness;
    bool triggered_qos;

    DDS_OfferedDeadlineMissedStatus status_deadline
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    DDS_OfferedIncompatibleQosStatus status_qos
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    DDS_LivelinessLostStatus status_liveliness
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    
    RMW_Connext_Publisher *pub;
};

void
    RMW_Connext_DataReaderListener_requested_deadline_missed(
        void *listener_data,
        DDS_DataReader *reader,
        const struct DDS_RequestedDeadlineMissedStatus *status);

void
RMW_Connext_DataReaderListener_requested_incompatible_qos(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_RequestedIncompatibleQosStatus *status);

void
RMW_Connext_DataReaderListener_liveliness_changed(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_LivelinessChangedStatus *status);

void
RMW_Connext_DataReaderListener_sample_lost(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SampleLostStatus *status);

void
RMW_Connext_DataReaderListener_on_data_available(
    void *listener_data,
    DDS_DataReader *reader);

class RMW_Connext_StdSubscriberStatusCondition : public RMW_Connext_StdStatusCondition
{
public:

    RMW_Connext_StdSubscriberStatusCondition()
    : triggered_deadline(false),
      triggered_liveliness(false),
      triggered_qos(false),
      triggered_sample_lost(false),
      triggered_data(false),
      listener_drop_handle(DDS_HANDLE_NIL),
      sub(nullptr)
    {

    }

    bool
    has_status(const rmw_event_type_t event_type)
    {
        switch (event_type)
        {
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
        #if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY
        case RMW_EVENT_MESSAGE_LOST:
        {
            return this->triggered_sample_lost;
        }
        #endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY */
        default:
            /* TODO assert unreachable */
            return false;
        }
    }

    bool
    get_status(const rmw_event_type_t event_type, void *const event_info)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        switch (event_type)
        {
        case RMW_EVENT_LIVELINESS_CHANGED:
        {
            rmw_liveliness_changed_status_t *status =
                (rmw_liveliness_changed_status_t*)event_info;
            
            status->alive_count = this->status_liveliness.alive_count;
            status->alive_count_change =
                this->status_liveliness.alive_count_change;
            status->not_alive_count =
                this->status_liveliness.not_alive_count;
            status->not_alive_count_change =
                this->status_liveliness.not_alive_count_change;

            this->status_liveliness.alive_count_change = 0;
            this->status_liveliness.not_alive_count_change = 0;
            this->triggered_liveliness = false;
            break;
        }
        case RMW_EVENT_REQUESTED_DEADLINE_MISSED:
        {
            rmw_requested_deadline_missed_status_t * status =
                (rmw_requested_deadline_missed_status_t *)event_info;
            
            status->total_count = this->status_deadline.total_count;
            status->total_count_change =
                this->status_deadline.total_count_change;
            
            this->status_deadline.total_count_change = 0;
            this->triggered_deadline = false;
            break;
        }
        #if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY
        case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
        {
            rmw_requested_qos_incompatible_event_status_t *const status =
                (rmw_requested_qos_incompatible_event_status_t*)event_info;
            
            status->total_count = this->status_qos.total_count;
            status->total_count_change =
                this->status_qos.total_count_change;
            status->last_policy_kind =
                dds_qos_policy_to_rmw_qos_policy(
                    this->status_qos.last_policy_id);
            
            this->status_qos.total_count_change = 0;
            this->triggered_qos = false;
            break;
        }
        case RMW_EVENT_MESSAGE_LOST:
        {
            rmw_message_lost_status_t *const status =
                (rmw_message_lost_status_t*)event_info;

            status->total_count = this->status_sample_lost.total_count;
            status->total_count_change =
                this->status_sample_lost.total_count_change;
            
            this->status_sample_lost.total_count_change = 0;
            this->triggered_sample_lost = false;
            break;
        }
        #endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY */
        default:
            /* TODO assert unreachable */
            return false;
        }

        return true;
    }

    friend
    void
    RMW_Connext_DataReaderListener_requested_deadline_missed(
        void *listener_data,
        DDS_DataReader *reader,
        const struct DDS_RequestedDeadlineMissedStatus *status);

    friend
    void
    RMW_Connext_DataReaderListener_requested_incompatible_qos(
        void *listener_data,
        DDS_DataReader *reader,
        const struct DDS_RequestedIncompatibleQosStatus *status);
    
    friend
    void
    RMW_Connext_DataReaderListener_liveliness_changed(
        void *listener_data,
        DDS_DataReader *reader,
        const struct DDS_LivelinessChangedStatus *status);

    friend
    void
    RMW_Connext_DataReaderListener_sample_lost(
        void *listener_data,
        DDS_DataReader *reader,
        const struct DDS_SampleLostStatus *status);
    
    friend
    void
    RMW_Connext_DataReaderListener_on_data_available(
        void *listener_data,
        DDS_DataReader *reader);
    
    rmw_ret_t
    install(RMW_Connext_Subscriber *const sub);

    bool
    on_data_triggered()
    {
        return this->triggered_data.exchange(false);
    }

    void
    on_data()
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->triggered_data = true;

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }
    

    void
    on_requested_deadline_missed(
        const DDS_RequestedDeadlineMissedStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_deadline(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

    void
    on_requested_incompatible_qos(
        const DDS_RequestedIncompatibleQosStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_qos(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

    void
    on_liveliness_changed(
        const DDS_LivelinessChangedStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_liveliness(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

    void
    on_sample_lost(
        const DDS_SampleLostStatus *const status)
    {
        std::lock_guard<std::mutex> lock(this->mutex_internal);

        this->update_status_sample_lost(status);

        if (nullptr != this->waitset_condition)
        {
            this->waitset_condition->notify_one();
        }
    }

    void set_drop_handle(const DDS_InstanceHandle_t handle)
    {
        this->listener_drop_handle = handle;
    }

    const DDS_InstanceHandle_t& drop_handle()
    {
        return this->listener_drop_handle;
    }

protected:
    void update_status_deadline(
            const DDS_RequestedDeadlineMissedStatus *const status)
    {
        this->status_deadline = *status;
        this->triggered_deadline = true;
    }

    void update_status_liveliness(
            const DDS_LivelinessChangedStatus *const status)
    {
        this->status_liveliness = *status;
        this->triggered_liveliness = true;
    }

    void update_status_qos(
            const DDS_RequestedIncompatibleQosStatus *const status)
    {
        this->status_qos = *status;
        this->triggered_qos = true;
    }

    void update_status_sample_lost(
            const DDS_SampleLostStatus *const status)
    {
        this->status_sample_lost = *status;
        this->triggered_sample_lost = true;
    }

    std::atomic_bool triggered_deadline;
    std::atomic_bool triggered_liveliness;
    std::atomic_bool triggered_qos;
    std::atomic_bool triggered_sample_lost;
    std::atomic_bool triggered_data;

    DDS_InstanceHandle_t listener_drop_handle;

    DDS_RequestedDeadlineMissedStatus status_deadline
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    DDS_RequestedIncompatibleQosStatus status_qos
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    DDS_LivelinessChangedStatus status_liveliness
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);
    DDS_SampleLostStatus status_sample_lost
        RCPPUTILS_TSA_GUARDED_BY(mutex_internal);

    RMW_Connext_Subscriber *sub;
};

/******************************************************************************
 * Node support
 ******************************************************************************/
class RMW_Connext_Node
{
    rmw_context_impl_t *ctx;

    RMW_Connext_Node(rmw_context_impl_t *const ctx)
    : ctx(ctx)
    {

    }

public:
    
    static
    RMW_Connext_Node*
    create(rmw_context_impl_t *const ctx);

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
    RMW_Connext_Publisher*
    create(
        rmw_context_impl_t *const ctx,
        DDS_DomainParticipant *const dp,
        DDS_Publisher *const pub,
        const rosidl_message_type_support_t *const type_supports,
        const char *const topic_name,
        const rmw_qos_profile_t *const qos_policies,
        const rmw_publisher_options_t *const publisher_options,
        const bool internal = false,
        const RMW_Connext_MessageType msg_type = RMW_CONNEXT_MESSAGE_USERDATA,
        const void *const intro_members = nullptr,
        const bool intro_members_cpp = false,
        std::string *const type_name = nullptr);
    
    rmw_ret_t
    finalize();

    DDS_DataWriter*
    writer() const
    {
        return this->dds_writer;
    }

    RMW_Connext_MessageTypeSupport*
    message_type_support() const
    {
        return this->type_support;
    }

    rmw_ret_t
    write(const void *const ros_message, const bool serialized);

    rmw_ret_t
    enable() const
    {
        if (DDS_RETCODE_OK !=
                DDS_Entity_enable(
                    DDS_Topic_as_entity(
                        DDS_DataWriter_get_topic(this->dds_writer))))
        {
            RMW_CONNEXT_LOG_ERROR("failed to enable dds writer's topic")
            return RMW_RET_ERROR;
        }

        if (DDS_RETCODE_OK !=
                DDS_Entity_enable(DDS_DataWriter_as_entity(this->dds_writer)))
        {
            RMW_CONNEXT_LOG_ERROR("failed to enable dds writer")
            return RMW_RET_ERROR;
        }

        return RMW_RET_OK;
    }

    DDS_Condition*
    condition() const
    {
        return this->dds_condition;
    }

    const rmw_gid_t* gid() const {
        return &this->ros_gid;
    }

    size_t
    subscriptions_count();

    rmw_ret_t
    assert_liveliness();

    rmw_ret_t
    qos(rmw_qos_profile_t *const qos);

    rmw_ret_t
    liveliness_lost_status(rmw_liveliness_lost_status_t *const status);

    rmw_ret_t
    offered_deadline_missed_status(
        rmw_offered_deadline_missed_status_t *const status);

#if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY
    rmw_ret_t
    offered_incompatible_qos_status(
        rmw_offered_qos_incompatible_event_status_t *const status);
#endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY */
    
    bool
    has_status(const DDS_StatusMask status_mask);

    rmw_ret_t
    enable_status(const DDS_StatusMask status_mask);
    
    rmw_ret_t
    disable_status(const DDS_StatusMask status_mask);

    bool
    has_status(const rmw_event_type_t event_type)
    {
        return this->std_condition.has_status(event_type);
    }

    bool
    get_status(const rmw_event_type_t event_type, void *const event_info)
    {
        return this->std_condition.get_status(event_type, event_info);
    }
    void
    attach(
        std::mutex *const waitset_mutex,
        std::condition_variable *const condition)
    {
        return this->std_condition.attach(waitset_mutex, condition);
    }

    void
    detach()
    {
        return this->std_condition.detach();
    }

    rmw_ret_t
    requestreply_header_to_dds(
        const RMW_Connext_RequestReplyMessage *const rr_msg,
        DDS_SampleIdentity_t *const sample_identity,
        DDS_SampleIdentity_t *const related_sample_identity);

private:
    rmw_context_impl_t *ctx;
    DDS_DataWriter *dds_writer;
    DDS_Condition *dds_condition;
    RMW_Connext_MessageTypeSupport *type_support;
    const bool created_topic;
    rmw_gid_t ros_gid;
    RMW_Connext_StdPublisherStatusCondition std_condition;

    RMW_Connext_Publisher(
        rmw_context_impl_t *const ctx,
        DDS_DataWriter *const dds_writer,
        RMW_Connext_MessageTypeSupport *const type_support,
        const bool created_topic);
    
    DDS_Topic* dds_topic()
    {
        return DDS_DataWriter_get_topic(this->dds_writer);
    }

    DDS_Publisher* dds_publisher()
    {
        return DDS_DataWriter_get_publisher(this->dds_writer);
    }

    DDS_DomainParticipant* dds_participant()
    {
        DDS_Publisher *const pub = this->dds_publisher();

        return DDS_Publisher_get_participant(pub);
    }

};


rmw_publisher_t *
rmw_connextdds_create_publisher(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    DDS_DomainParticipant *const dp,
    DDS_Publisher *const pub,
    const rosidl_message_type_support_t *const type_supports,
    const char *const topic_name,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_publisher_options_t *const publisher_options,
    const bool internal = false);

rmw_ret_t
rmw_connextdds_destroy_publisher(
    rmw_context_impl_t *const ctx,
    rmw_publisher_t *const pub);

/******************************************************************************
 * Subscription support
 ******************************************************************************/

class RMW_Connext_Subscriber
{
public:

    static
    RMW_Connext_Subscriber*
    create(
        rmw_context_impl_t *const ctx,
        DDS_DomainParticipant *const dp,
        DDS_Subscriber *const sub,
        const rosidl_message_type_support_t *const type_supports,
        const char *const topic_name,
        const rmw_qos_profile_t *const qos_policies,
        const rmw_subscription_options_t *const subscriber_options,
        const bool internal = false,
        const RMW_Connext_MessageType msg_type = RMW_CONNEXT_MESSAGE_USERDATA,
        const void *const intro_members = nullptr,
        const bool intro_members_cpp = false,
        std::string *const type_name = nullptr);
    
    rmw_ret_t
    finalize();

    bool
    ignore_local() const
    {
        return this->opt_ignore_local;
    }

    DDS_DataReader*
    reader() const
    {
        return this->dds_reader;
    }

    RMW_Connext_UntypedSampleSeq*
    data_seq()
    {
        return &this->loan_data;
    }
    DDS_SampleInfoSeq*
    info_seq()
    {
        return &this->loan_info;
    }

    RMW_Connext_MessageTypeSupport*
    message_type_support() const
    {
        return this->type_support;
    }

    DDS_InstanceHandle_t
    instance_handle()
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
            RMW_CONNEXT_LOG_ERROR("failed to enable dds reader's topic")
            return RMW_RET_ERROR;
        }

        if (DDS_RETCODE_OK !=
                DDS_Entity_enable(DDS_DataReader_as_entity(this->dds_reader)))
        {
            RMW_CONNEXT_LOG_ERROR("failed to enable dds reader")
            return RMW_RET_ERROR;
        }

        return RMW_RET_OK;
    }

    DDS_Condition*
    condition() const
    {
        return this->dds_condition;
    }

    const rmw_gid_t* gid() const
    {
        return &this->ros_gid;
    }

    size_t
    publications_count();

    rmw_ret_t
    qos(rmw_qos_profile_t *const qos);

    void
    loan_begin()
    {
        std::unique_lock<std::mutex> lock(this->loan_mutex);
        auto on_condition = [sub = this]() { return !sub->loan_active; };
        this->loan_condition.wait(lock, on_condition);
        this->loan_active = true;
    }

    void
    loan_end()
    {
        std::unique_lock<std::mutex> lock(this->loan_mutex);
        this->loan_active = false;
        this->loan_condition.notify_all();
    }

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
        if (this->loan_len == 0 || this->loan_next >= this->loan_len)
        {
            /* return any previously loaned messages */
            if (this->loan_len > 0)
            {
                rc = this->return_messages();
                if (RMW_RET_OK != rc)
                {
                    goto done;
                }
            }
            /* loan messages from reader */
            rc = this->loan_messages();
        }

    done:
        return rc;
    }

    void
    requestreply_header_from_dds(
        RMW_Connext_RequestReplyMessage *const rr_msg,
        const DDS_SampleIdentity_t *const sample_identity,
        const DDS_SampleIdentity_t *const related_sample_identity);

    rmw_ret_t
    take_next(
        void **const ros_messages,
        rmw_message_info_t *const message_infos,
        const size_t max_samples,
        size_t *const taken,
        const bool serialized);

    rmw_ret_t
    take_message(
        void *const ros_message,
        rmw_message_info_t *const message_info,
        bool *const taken);
    
    rmw_ret_t
    take(
        rmw_message_sequence_t *const message_sequence,
        rmw_message_info_sequence_t *const message_info_sequence,
        const size_t max_samples,
        size_t *const taken);
    
    rmw_ret_t
    take_serialized(
        rmw_serialized_message_t *const serialized_message,
        rmw_message_info_t *const message_info,
        bool *const taken);

    rmw_ret_t
    liveliness_changed_status(rmw_liveliness_changed_status_t *const status);

    rmw_ret_t
    requested_deadline_missed_status(
        rmw_requested_deadline_missed_status_t *const status);

#if RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY
    rmw_ret_t
    requested_incompatible_qos_status(
        rmw_requested_qos_incompatible_event_status_t *const status);
    
    rmw_ret_t
    message_lost_status(rmw_message_lost_status_t *const status);
#endif /* RMW_CONNEXT_RELEASE != RMW_CONNEXT_RELEASE_FOXY */

    bool
    has_status(const DDS_StatusMask status_mask);

    rmw_ret_t
    enable_status(const DDS_StatusMask status_mask);

    rmw_ret_t
    disable_status(const DDS_StatusMask status_mask);

    bool
    has_data()
    {
        // check (and reset) flag for on_data_available callback
        const bool on_data_triggered = this->std_condition.on_data_triggered();

        std::lock_guard<std::mutex> lock(this->loan_mutex);
        // this->loan_begin();

        if (on_data_triggered)
        {
            this->loan_messages_if_needed();
        }
        bool has_data = this->loan_len > 0;
        RMW_CONNEXT_ASSERT(!has_data || this->loan_next < this->loan_len)

        // this->loan_end();

        return has_data;
    }

    bool
    has_status(const rmw_event_type_t event_type)
    {
        return this->std_condition.has_status(event_type);
    }

    bool
    get_status(const rmw_event_type_t event_type, void *const event_info)
    {
        return this->std_condition.get_status(event_type, event_info);
    }

    void
    attach(
        std::mutex *const waitset_mutex,
        std::condition_variable *const condition)
    {
        return this->std_condition.attach(waitset_mutex, condition);
    }

    void
    detach()
    {
        return this->std_condition.detach();
    }

private:
    rmw_context_impl_t *ctx;
    DDS_DataReader *dds_reader;
    DDS_Topic *dds_topic;
    DDS_Condition *dds_condition;
    RMW_Connext_MessageTypeSupport *type_support;
    rmw_gid_t ros_gid;
    const bool opt_ignore_local;
    const bool created_topic;
    RMW_Connext_StdSubscriberStatusCondition std_condition;
    RMW_Connext_UntypedSampleSeq loan_data;
    DDS_SampleInfoSeq loan_info;
    size_t loan_len;
    size_t loan_next;
    std::mutex loan_mutex;
    std::condition_variable loan_condition;
    bool loan_active;

    RMW_Connext_Subscriber(
        rmw_context_impl_t *const ctx,
        DDS_DataReader *const dds_reader,
        DDS_Topic *const dds_topic,
        RMW_Connext_MessageTypeSupport *const type_support,
        const bool ignore_local,
        const bool created_topic);
    
    DDS_Subscriber* dds_subscriber()
    {
        return DDS_DataReader_get_subscriber(this->dds_reader);
    }

    DDS_DomainParticipant* dds_participant()
    {
        DDS_Subscriber *const sub = this->dds_subscriber();

        return DDS_Subscriber_get_participant(sub);
    }
};

rmw_subscription_t *
rmw_connextdds_create_subscriber(
    rmw_context_impl_t *const ctx,
    const rmw_node_t *const node,
    DDS_DomainParticipant *const dp,
    DDS_Subscriber *const sub,
    const rosidl_message_type_support_t *const type_supports,
    const char *const topic_name,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_subscription_options_t *const subscriber_options,
    const bool internal = false);

rmw_ret_t
rmw_connextdds_destroy_subscriber(
    rmw_context_impl_t *const ctx,
    rmw_subscription_t *const sub);

void
rmw_connextdds_message_info_from_dds(
    rmw_message_info_t *const to,
    const DDS_SampleInfo *const from);

/******************************************************************************
 * Client/Service support
 ******************************************************************************/

class RMW_Connext_Client
{
    RMW_Connext_Publisher *request_pub;
    RMW_Connext_Subscriber *reply_sub;
    int64_t next_request_id;

    RMW_Connext_Client()
    : request_pub(nullptr),
      reply_sub(nullptr),
      next_request_id(1)
    {
        
    }

public:
    static
    RMW_Connext_Client*
    create(
        rmw_context_impl_t *const ctx,
        DDS_DomainParticipant *const dp,
        DDS_Publisher *const pub,
        DDS_Subscriber *const sub,
        const rosidl_service_type_support_t *const type_supports,
        const char *const svc_name,
        const rmw_qos_profile_t *const qos_policies);
    
    rmw_ret_t
    finalize();

    rmw_ret_t
    is_service_available(bool &available);

    rmw_ret_t
    take_response(
        rmw_service_info_t *const request_header,
        void *const ros_response,
        bool *const taken);
    
    rmw_ret_t
    send_request(
        const void *const ros_request,
        int64_t *const sequence_id);
    
    RMW_Connext_Publisher*
    publisher() const
    {
        return this->request_pub;
    }

    RMW_Connext_Subscriber*
    subscriber() const
    {
        return this->reply_sub;
    }
};

class RMW_Connext_Service
{
    RMW_Connext_Publisher *reply_pub;
    RMW_Connext_Subscriber *request_sub;

public:
    static
    RMW_Connext_Service*
    create(
        rmw_context_impl_t *const ctx,
        DDS_DomainParticipant *const dp,
        DDS_Publisher *const pub,
        DDS_Subscriber *const sub,
        const rosidl_service_type_support_t *const type_supports,
        const char *const svc_name,
        const rmw_qos_profile_t *const qos_policies);
    
    rmw_ret_t
    finalize();

    rmw_ret_t
    take_request(
        rmw_service_info_t *const request_header,
        void *const ros_request,
        bool *const taken);
    
    rmw_ret_t
    send_response(
        rmw_request_id_t *const request_id,
        const void *const ros_response);
    
    RMW_Connext_Publisher*
    publisher() const
    {
        return this->reply_pub;
    }

    RMW_Connext_Subscriber*
    subscriber() const
    {
        return this->request_sub;
    }
};


/******************************************************************************
 * Event support
 ******************************************************************************/


bool
ros_event_for_reader(const rmw_event_type_t ros);

DDS_StatusKind
ros_event_to_dds(const rmw_event_type_t ros, bool *const invalid);

const char*
dds_event_to_str(const DDS_StatusKind event);

class RMW_Connext_Event
{
public:
    
    static
    rmw_ret_t
    enable(rmw_event_t *const event);

    static
    rmw_ret_t
    disable(rmw_event_t *const event);

    static
    bool
    active(rmw_event_t *const event);

    static
    DDS_Condition*
    condition(const rmw_event_t *const event)
    {
        if (RMW_Connext_Event::reader_event(event))
        {
            return RMW_Connext_Event::subscriber(event)->condition();
        }
        else
        {
            return RMW_Connext_Event::publisher(event)->condition();
        }
    }

    static
    bool
    writer_event(const rmw_event_t *const event)
    {
        return !ros_event_for_reader(event->event_type);
    }

    static
    bool
    reader_event(const rmw_event_t *const event)
    {
        return ros_event_for_reader(event->event_type);
    }

    static
    RMW_Connext_Publisher*
    publisher(const rmw_event_t *const event)
    {
        return (RMW_Connext_Publisher*)event->data;
    }

    static
    RMW_Connext_Subscriber*
    subscriber(const rmw_event_t *const event)
    {
        return (RMW_Connext_Subscriber*)event->data;
    }
};


/******************************************************************************
 * Waitsets and Conditions
 ******************************************************************************/

rmw_guard_condition_t *
rmw_connextdds_create_guard_condition(const bool internal = false);

rmw_ret_t
rmw_connextdds_destroy_guard_condition(rmw_guard_condition_t *const gc);

rmw_ret_t
rmw_connextdds_trigger_guard_condition(const rmw_guard_condition_t *const gc);


class RMW_Connext_WaitSet
{
    DDS_WaitSet *waitset;
    std::mutex waiting_lock;
    bool waiting;
    std::vector<RMW_Connext_Subscriber *> attached_subscribers;
    std::vector<DDS_GuardCondition *> attached_conditions;
    std::vector<RMW_Connext_Client *> attached_clients;
    std::vector<RMW_Connext_Service *> attached_services;
    std::vector<rmw_event_t *> attached_events;
    std::vector<RMW_Connext_Subscriber *> attached_event_subscribers;
    std::vector<RMW_Connext_Publisher *> attached_event_publishers;
    
    struct DDS_ConditionSeq active_conditions;

    template<typename T>
    bool
    require_attach(
        const std::vector<T *> &attached_els,
        const size_t new_els_count,
        void **const new_els);
    
    rmw_ret_t
    attach(
        rmw_subscriptions_t * subs,
        rmw_guard_conditions_t * gcs,
        rmw_services_t * srvs,
        rmw_clients_t * cls,
        rmw_events_t * evs);
    
    rmw_ret_t
    detach();

public:

    static
    RMW_Connext_WaitSet*
    create();

    rmw_ret_t
    finalize();

    rmw_ret_t
    wait(rmw_subscriptions_t * subs,
        rmw_guard_conditions_t * gcs,
        rmw_services_t * srvs,
        rmw_clients_t * cls,
        rmw_events_t * evs,
        const rmw_time_t * wait_timeout);
};

rmw_wait_set_t *
rmw_connextdds_create_waitset(const size_t max_conditions);

rmw_ret_t
rmw_connextdds_destroy_waitset(rmw_wait_set_t *const ws);

rmw_ret_t
rmw_connextdds_waitset_wait(
    rmw_wait_set_t *const ws,
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
rmw_connextdds_gid_to_guid(const rmw_gid_t &gid, struct DDS_GUID_t &guid);

rmw_ret_t
rmw_connextdds_guid_to_gid(const struct DDS_GUID_t &guid, rmw_gid_t &gid);

void
rmw_connextdds_sn_ros_to_dds(
    const int64_t sn_ros, struct DDS_SequenceNumber_t &sn_dds);

#define rmw_connextdds_sn_ros_to_dds(sn_ros_, sn_dds_)\
{\
    (sn_dds_).high = ((sn_ros_) & 0xFFFFFFFF00000000LL) >> 32;\
    (sn_dds_).low = (sn_ros_) & 0x00000000FFFFFFFFLL;\
}

void
rmw_connextdds_sn_dds_to_ros(
    const struct DDS_SequenceNumber_t &sn_dds, int64_t &sn_ros);

#define rmw_connextdds_sn_dds_to_ros(sn_dds_, sn_ros_)\
{\
    sn_ros_ = ((int64_t)(sn_dds_).high) << 32 | (sn_dds_).low;\
}

void rmw_connextdds_ih_to_gid(
    const DDS_InstanceHandle_t &ih, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_Entity *const entity, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_DomainParticipant *const dp, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_Publisher *const pub, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_Subscriber *const sub, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_DataWriter *const writer, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_DataReader *const reader, rmw_gid_t &gid);

void rmw_connextdds_get_entity_gid(
    DDS_Topic *const topic, rmw_gid_t &gid);

/******************************************************************************
 * Type helpers
 ******************************************************************************/
std::string
rmw_connextdds_create_type_name(
    const message_type_support_callbacks_t * callbacks,
    const bool mangle_names = true);

std::string
rmw_connextdds_create_type_name(
    const rosidl_typesupport_introspection_cpp::MessageMembers *const members,
    const bool mangle_names = true);

std::string
rmw_connextdds_create_type_name(
    const rosidl_typesupport_introspection_c__MessageMembers *const members,
    const bool mangle_names = true);

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
    RMW_Connext_MessageTypeSupport *const type_support,
    DDS_HistoryQosPolicy *const history,
    DDS_ReliabilityQosPolicy *const reliability,
    DDS_DurabilityQosPolicy *const durability,
    DDS_DeadlineQosPolicy *const deadline,
    DDS_LivelinessQosPolicy *const liveliness,
    DDS_ResourceLimitsQosPolicy *const resource_limits,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_publisher_options_t *const pub_options,
    const rmw_subscription_options_t *const sub_options);

rmw_ret_t
rmw_connextdds_readerwriter_qos_to_ros(
    const DDS_HistoryQosPolicy *const history,
    const DDS_ReliabilityQosPolicy *const reliability,
    const DDS_DurabilityQosPolicy *const durability,
    const DDS_DeadlineQosPolicy *const deadline,
    const DDS_LivelinessQosPolicy *const liveliness,
    rmw_qos_profile_t *const qos_policies);

#endif /* RMW_CONNEXT__RMW_IMPL_HPP_ */