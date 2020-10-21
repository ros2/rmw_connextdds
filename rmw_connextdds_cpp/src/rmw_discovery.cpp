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

#include "rmw_connextdds_cpp/rmw_impl.hpp"
#include "rmw_connextdds_cpp/discovery.hpp"
#include "rmw_connextdds_cpp/graph_cache.hpp"

/******************************************************************************
 * Discovery Thread
 ******************************************************************************/
#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO
static
void
rmw_connextdds_discovery_thread_on_dcps_participant(
    rmw_context_impl_t * ctx)
{
    DDS_ParticipantBuiltinTopicDataSeq data_seq = DDS_SEQUENCE_INITIALIZER;
    DDS_SampleInfoSeq info_seq = DDS_SEQUENCE_INITIALIZER;
    DDS_ReturnCode_t rc = DDS_RETCODE_OK;

    DDS_ParticipantBuiltinTopicDataDataReader *const reader =
        DDS_ParticipantBuiltinTopicDataDataReader_narrow(ctx->dr_participants);

    do
    {
        rc = DDS_ParticipantBuiltinTopicDataDataReader_take(
                reader,
                &data_seq,
                &info_seq,
                DDS_LENGTH_UNLIMITED,
                DDS_ANY_SAMPLE_STATE,
                DDS_ANY_VIEW_STATE,
                DDS_ANY_INSTANCE_STATE);
        if (DDS_RETCODE_OK != rc)
        {
            continue;
        }

        const size_t data_len =
            DDS_ParticipantBuiltinTopicDataSeq_get_length(&data_seq);
        for (size_t i = 0; i < data_len; i++)
        {
            DDS_ParticipantBuiltinTopicData *const data = 
                DDS_ParticipantBuiltinTopicDataSeq_get_reference(&data_seq, i);
            DDS_SampleInfo *const info =
                DDS_SampleInfoSeq_get_reference(&info_seq, i);
            
            if (!info->valid_data)
            {
                /* TODO Check for instance_state != ALIVE to remove the remote
                   participant from the graph_cache by calling:
                   graph_cache.remove_participant(gid) */
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] ignored participant invalid data")
                continue;
            }

            DDS_GUID_t dp_guid;
            rmw_gid_t gid;
            DDS_BuiltinTopicKey_to_guid(&data->key, &dp_guid);
            rmw_connextdds_guid_to_gid(dp_guid, gid);

            if (0 ==
                memcmp(gid.data, ctx->common.gid.data, RMW_GID_STORAGE_SIZE))
            {
                /* Ignore own announcements */
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] ignored own participant data")
                continue;
            }
            
            std::string enclave;
            /* TODO retrieve enclave from USER_DATA */

            RMW_CONNEXT_LOG_DEBUG_A(
                "[discovery thread] assert participant: "
                "gid=0x%08X.0x%08X.0x%08X.0x%08X",
                ((uint32_t*)dp_guid.value)[0],
                ((uint32_t*)dp_guid.value)[1],
                ((uint32_t*)dp_guid.value)[2],
                ((uint32_t*)dp_guid.value)[3])

            std::lock_guard<std::mutex> guard(ctx->common.node_update_mutex);
            ctx->common.graph_cache.add_participant(gid, enclave);
        }

        if (DDS_RETCODE_OK !=
                DDS_ParticipantBuiltinTopicDataDataReader_return_loan(
                    reader, &data_seq, &info_seq))
        {
            RMW_CONNEXT_LOG_ERROR("failed to return loan to dds reader")
            return;
        }
        
    } while (DDS_RETCODE_OK == rc);
}
static
void
rmw_connextdds_discovery_add_entity_to_graph(
    rmw_context_impl_t * ctx,
    const DDS_GUID_t *const endp_guid,
    const DDS_GUID_t *const dp_guid,
    const char *const topic_name,
    const char *const type_name,
    const DDS_ReliabilityQosPolicy *const reliability,
    const DDS_DurabilityQosPolicy *const durability,
    const DDS_DeadlineQosPolicy *const deadline,
    const DDS_LivelinessQosPolicy *const liveliness,
    const bool is_reader)
{
    rmw_gid_t gid;
    rmw_gid_t dp_gid;
    rmw_connextdds_guid_to_gid(*endp_guid, gid);
    rmw_connextdds_guid_to_gid(*dp_guid, dp_gid);

    if (0 == memcmp(dp_gid.data, ctx->common.gid.data, RMW_GID_STORAGE_SIZE))
    {
        /* Ignore own announcements */
        return;
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_unknown;

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
        ((uint32_t*)dp_guid->value)[0],
        ((uint32_t*)dp_guid->value)[1],
        ((uint32_t*)dp_guid->value)[2],
        ((uint32_t*)dp_guid->value)[3],
        ((uint32_t*)endp_guid->value)[0],
        ((uint32_t*)endp_guid->value)[1],
        ((uint32_t*)endp_guid->value)[2],
        ((uint32_t*)endp_guid->value)[3],
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

static
void
rmw_connextdds_discovery_thread_on_dcps_subscription(
    rmw_context_impl_t * ctx)
{
    DDS_SubscriptionBuiltinTopicDataSeq data_seq = DDS_SEQUENCE_INITIALIZER;
    DDS_SampleInfoSeq info_seq = DDS_SEQUENCE_INITIALIZER;
    DDS_ReturnCode_t rc = DDS_RETCODE_OK;

    DDS_SubscriptionBuiltinTopicDataDataReader *const reader =
        DDS_SubscriptionBuiltinTopicDataDataReader_narrow(
            ctx->dr_subscriptions);

    do
    {
        rc = DDS_SubscriptionBuiltinTopicDataDataReader_take(
                reader,
                &data_seq,
                &info_seq,
                DDS_LENGTH_UNLIMITED,
                DDS_ANY_SAMPLE_STATE,
                DDS_ANY_VIEW_STATE,
                DDS_ANY_INSTANCE_STATE);
        if (DDS_RETCODE_OK != rc)
        {
            continue;
        }

        const size_t data_len =
            DDS_SubscriptionBuiltinTopicDataSeq_get_length(&data_seq);
        for (size_t i = 0; i < data_len; i++)
        {
            DDS_SubscriptionBuiltinTopicData *const data = 
                DDS_SubscriptionBuiltinTopicDataSeq_get_reference(&data_seq, i);
            DDS_SampleInfo *const info =
                DDS_SampleInfoSeq_get_reference(&info_seq, i);
            
            if (!info->valid_data)
            {
                /* TODO Check for instance_state != ALIVE to remove the remove
                   endpoint from the graph_cache by calling:
                   graph_cache.remove_entity(gid, is_reader = true) */
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] ignored subscription invalid data")
                continue;
            }

            // if (0 ==
            //     memcmp(data->key.value, ctx->common.gid.data, 12))
            // {
            //     /* Ignore own announcements */
            //     RMW_CONNEXT_LOG_DEBUG(
            //         "[discovery thread] ignored own subscription data")
            //     continue;
            // }

            DDS_GUID_t endp_guid;
            DDS_GUID_t dp_guid;

            DDS_BuiltinTopicKey_to_guid(&data->key, &endp_guid);
            DDS_BuiltinTopicKey_to_guid(&data->participant_key, &dp_guid);

            rmw_connextdds_discovery_add_entity_to_graph(
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
            return;
        }
        
    } while (DDS_RETCODE_OK == rc);
}

static
void
rmw_connextdds_discovery_thread_on_dcps_publication(
    rmw_context_impl_t * ctx)
{
    DDS_PublicationBuiltinTopicDataSeq data_seq = DDS_SEQUENCE_INITIALIZER;
    DDS_SampleInfoSeq info_seq = DDS_SEQUENCE_INITIALIZER;
    DDS_ReturnCode_t rc = DDS_RETCODE_OK;

    DDS_PublicationBuiltinTopicDataDataReader *const reader =
        DDS_PublicationBuiltinTopicDataDataReader_narrow(ctx->dr_publications);

    do
    {
        rc = DDS_PublicationBuiltinTopicDataDataReader_take(
                reader,
                &data_seq,
                &info_seq,
                DDS_LENGTH_UNLIMITED,
                DDS_ANY_SAMPLE_STATE,
                DDS_ANY_VIEW_STATE,
                DDS_ANY_INSTANCE_STATE);
        if (DDS_RETCODE_OK != rc)
        {
            continue;
        }

        const size_t data_len =
            DDS_PublicationBuiltinTopicDataSeq_get_length(&data_seq);
        for (size_t i = 0; i < data_len; i++)
        {
            DDS_PublicationBuiltinTopicData *const data = 
                DDS_PublicationBuiltinTopicDataSeq_get_reference(&data_seq, i);
            DDS_SampleInfo *const info =
                DDS_SampleInfoSeq_get_reference(&info_seq, i);
            
            if (!info->valid_data)
            {
                /* TODO Check for instance_state != ALIVE to remove the remove
                   endpoint from the graph_cache by calling:
                   graph_cache.remove_entity(gid, is_reader = false) */
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] ignored publication invalid data")
                continue;
            }

            // if (0 ==
            //     memcmp(data->key.value, ctx->common.gid.data, 12))
            // {
            //     /* Ignore own announcements */
            //     RMW_CONNEXT_LOG_DEBUG(
            //         "[discovery thread] ignored own publication data")
            //     continue;
            // }

            DDS_GUID_t endp_guid;
            DDS_GUID_t dp_guid;

            DDS_BuiltinTopicKey_to_guid(&data->key, &endp_guid);
            DDS_BuiltinTopicKey_to_guid(&data->participant_key, &dp_guid);

            rmw_connextdds_discovery_add_entity_to_graph(
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
            return;
        }
        
    } while (DDS_RETCODE_OK == rc);
}
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */

#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO

#ifndef RMW_CONNEXT_USE_BUILTIN_LISTENER
#define RMW_CONNEXT_USE_BUILTIN_LISTENER            0
#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */

#if RMW_CONNEXT_USE_BUILTIN_LISTENER
static
void rmw_connextdds_discovery_thread_dcps_participant_on_data_available(
        void *listener_data,
        DDS_DataReader *reader)
{
    rmw_context_impl_t *const ctx = (rmw_context_impl_t*)listener_data;
    UNUSED_ARG(reader);
    rmw_connextdds_discovery_thread_on_dcps_participant(ctx);
}

static
void rmw_connextdds_discovery_thread_dcps_subscription_on_data_available(
        void *listener_data,
        DDS_DataReader *reader)
{
    rmw_context_impl_t *const ctx = (rmw_context_impl_t*)listener_data;
    UNUSED_ARG(reader);
    rmw_connextdds_discovery_thread_on_dcps_subscription(ctx);
}

static
void rmw_connextdds_discovery_thread_dcps_publication_on_data_available(
        void *listener_data,
        DDS_DataReader *reader)
{
    rmw_context_impl_t *const ctx = (rmw_context_impl_t*)listener_data;
    UNUSED_ARG(reader);
    rmw_connextdds_discovery_thread_on_dcps_publication(ctx);
}

#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */

DDS_DataReader *
rmw_connextdds_get_builtin_reader_participants(rmw_context_impl_t * ctx)
{
    DDS_Subscriber *const sub =
        DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
    if (nullptr == sub)
    {
        return nullptr;
    }

    DDS_DataReader *const reader =
        DDS_Subscriber_lookup_datareader(sub, DDS_PARTICIPANT_TOPIC_NAME);
    
#if RMW_CONNEXT_USE_BUILTIN_LISTENER
    if (reader == nullptr)
    {
        return nullptr;
    }
    DDS_DataReaderListener builtin_listener =
        DDS_DataReaderListener_INITIALIZER;
    builtin_listener.as_listener.listener_data = ctx;
    builtin_listener.on_data_available =
        rmw_connextdds_discovery_thread_dcps_participant_on_data_available;
    if (DDS_RETCODE_OK !=
            DDS_DataReader_set_listener(
                reader, &builtin_listener, DDS_DATA_AVAILABLE_STATUS))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to set builtin reader listener (participants)")
        return nullptr;
    }
#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */

    return reader;
}

DDS_DataReader *
rmw_connextdds_get_builtin_reader_publications(rmw_context_impl_t * ctx)
{
    DDS_Subscriber *const sub =
        DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
    if (nullptr == sub)
    {
        return nullptr;
    }

    DDS_DataReader *const reader =
        DDS_Subscriber_lookup_datareader(sub, DDS_PUBLICATION_TOPIC_NAME);

#if RMW_CONNEXT_USE_BUILTIN_LISTENER
    if (reader == nullptr)
    {
        return nullptr;
    }
    DDS_DataReaderListener builtin_listener =
        DDS_DataReaderListener_INITIALIZER;
    builtin_listener.as_listener.listener_data = ctx;
    builtin_listener.on_data_available =
        rmw_connextdds_discovery_thread_dcps_publication_on_data_available;
    if (DDS_RETCODE_OK !=
            DDS_DataReader_set_listener(
                reader, &builtin_listener, DDS_DATA_AVAILABLE_STATUS))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to set builtin reader listener (publications)")
        return nullptr;
    }
#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */
    
    return reader;
}

DDS_DataReader *
rmw_connextdds_get_builtin_reader_subscriptions(rmw_context_impl_t * ctx)
{
    DDS_Subscriber *const sub =
        DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
    if (nullptr == sub)
    {
        return nullptr;
    }

    DDS_DataReader *const reader =
        DDS_Subscriber_lookup_datareader(sub, DDS_SUBSCRIPTION_TOPIC_NAME);

#if RMW_CONNEXT_USE_BUILTIN_LISTENER
    if (reader == nullptr)
    {
        return nullptr;
    }
    DDS_DataReaderListener builtin_listener =
        DDS_DataReaderListener_INITIALIZER;
    builtin_listener.as_listener.listener_data = ctx;
    builtin_listener.on_data_available =
        rmw_connextdds_discovery_thread_dcps_subscription_on_data_available;
    if (DDS_RETCODE_OK !=
            DDS_DataReader_set_listener(
                reader, &builtin_listener, DDS_DATA_AVAILABLE_STATUS))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to set builtin reader listener (subscriptions)")
        return nullptr;
    }
#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */

    return reader;
}


rmw_ret_t
rmw_connextdds_enable_builtin_readers(rmw_context_impl_t * ctx)
{
    /* Enable builtin subscriber and endpoints */
    DDS_Subscriber *const sub =
        DDS_DomainParticipant_get_builtin_subscriber(ctx->participant);
    if (nullptr == sub)
    {
        return RMW_RET_ERROR;
    }

    if (DDS_RETCODE_OK != DDS_Entity_enable(DDS_Subscriber_as_entity(sub)))
    {
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

#elif RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO

DDS_DataReader *
rmw_connextdds_get_builtin_reader_participants(rmw_context_impl_t * ctx)
{
    UNUSED_ARG(ctx);
    return nullptr;
}

DDS_DataReader *
rmw_connextdds_get_builtin_reader_publications(rmw_context_impl_t * ctx)
{
    UNUSED_ARG(ctx);
    return nullptr;
}

DDS_DataReader *
rmw_connextdds_get_builtin_reader_subscriptions(rmw_context_impl_t * ctx)
{
    UNUSED_ARG(ctx);
    return nullptr;
}


rmw_ret_t
rmw_connextdds_enable_builtin_readers(rmw_context_impl_t * ctx)
{
    UNUSED_ARG(ctx);
    return RMW_RET_OK;
}
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */


static
void
rmw_connextdds_discovery_thread(rmw_context_impl_t * ctx)
{
    RMW_CONNEXT_LOG_DEBUG("[discovery thread] starting up...")
    
    RMW_Connext_Subscriber *const sub_partinfo =
        (RMW_Connext_Subscriber*)ctx->common.sub->data;

    DDS_ConditionSeq active_conditions = DDS_SEQUENCE_INITIALIZER;
    DDS_ReturnCode_t rc = DDS_RETCODE_ERROR;
    DDS_UnsignedLong active_len = 0,
                     i = 0;

    bool attached_exit = false,
         attached_partinfo = false;
    
    RMW_Connext_StdGuardCondition *const gcond_exit =
        (RMW_Connext_StdGuardCondition*)ctx->common.listener_thread_gc->data;   
    DDS_Condition *cond_exit =
        DDS_GuardCondition_as_condition(gcond_exit->guard_condition());

    DDS_Condition *cond_partinfo = sub_partinfo->condition(),
                  *cond_active = nullptr;
#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO
#if !RMW_CONNEXT_USE_BUILTIN_LISTENER
    bool attached_dcps_part = false,
         attached_dcps_sub = false,
         attached_dcps_pub = false;
    
    DDS_StatusMask builtin_cond_mask = DDS_DATA_AVAILABLE_STATUS;
    DDS_StatusCondition *cond_dcps_part =
                            DDS_Entity_get_statuscondition(
                                DDS_DataReader_as_entity(ctx->dr_participants)),
                        *cond_dcps_pub = 
                            DDS_Entity_get_statuscondition(
                                DDS_DataReader_as_entity(ctx->dr_publications)),
                        *cond_dcps_sub = 
                            DDS_Entity_get_statuscondition(
                                DDS_DataReader_as_entity(
                                    ctx->dr_subscriptions));
#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */
    
    DDS_WaitSet *waitset = DDS_WaitSet_new();
    if (nullptr == waitset)
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to create waitset for discovery thread")
        return;
    }

    if (DDS_RETCODE_OK !=
            DDS_WaitSet_attach_condition(waitset, cond_exit))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to attach exit condition to discovery thread waitset")
        goto cleanup;
    }
    attached_exit = true;

    if (DDS_RETCODE_OK != DDS_WaitSet_attach_condition(waitset, cond_partinfo))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to attach participant info condition to "
            "discovery thread waitset")
        goto cleanup;
    }
    attached_partinfo = true;
    
#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO
#if !RMW_CONNEXT_USE_BUILTIN_LISTENER
    if (DDS_RETCODE_OK !=
            DDS_StatusCondition_set_enabled_statuses(
                cond_dcps_part, builtin_cond_mask))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set datareader condition mask")
        goto cleanup;
    }

    if (DDS_RETCODE_OK !=
            DDS_WaitSet_attach_condition(waitset,
                DDS_StatusCondition_as_condition(cond_dcps_part)))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to attach DCPS Participant condition to "
            "discovery thread waitset")
        goto cleanup;
    }
    attached_dcps_part = true;

    if (DDS_RETCODE_OK !=
            DDS_StatusCondition_set_enabled_statuses(
                cond_dcps_sub, builtin_cond_mask))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set datareader condition mask")
        goto cleanup;
    }

    if (DDS_RETCODE_OK !=
            DDS_WaitSet_attach_condition(waitset,
                DDS_StatusCondition_as_condition(cond_dcps_sub)))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to attach DCPS Subscription condition to "
            "discovery thread waitset")
        goto cleanup;
    }
    attached_dcps_sub = true;

    if (DDS_RETCODE_OK !=
            DDS_StatusCondition_set_enabled_statuses(
                cond_dcps_pub, builtin_cond_mask))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set datareader condition mask")
        goto cleanup;
    }

    if (DDS_RETCODE_OK !=
            DDS_WaitSet_attach_condition(waitset,
                DDS_StatusCondition_as_condition(cond_dcps_pub)))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to attach DCPS Publication condition to "
            "discovery thread waitset")
        goto cleanup;
    }
    attached_dcps_pub = true;
#endif /* RMW_CONNEXT_USE_BUILTIN_LISTENER */

#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */
    
    if (!DDS_ConditionSeq_set_maximum(&active_conditions, 5))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set condition seq maximum")
        goto cleanup;
    }

    RMW_CONNEXT_LOG_DEBUG("[discovery thread] main loop")

    while (ctx->common.thread_is_running.load())
    {
        RMW_CONNEXT_LOG_TRACE("[discovery thread] waiting...")
        rc = DDS_WaitSet_wait(
                waitset, &active_conditions, &DDS_DURATION_INFINITE);
        
        if (DDS_RETCODE_OK != rc)
        {
            RMW_CONNEXT_LOG_ERROR("wait failed for discovery thread")
            goto cleanup;
        }

        active_len = DDS_ConditionSeq_get_length(&active_conditions);
        
        RMW_CONNEXT_LOG_TRACE_A(
            "[discovery thread] active=%u", active_len)

        for (i = 0; i < active_len; i++)
        {
            cond_active =
                *DDS_ConditionSeq_get_reference(&active_conditions, i);
            if (cond_exit == cond_active)
            {
                /* nothing to do */
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] exit condition active")
            }
            else if (cond_partinfo == cond_active)
            {
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] participant-info active")
                rmw_connextdds_graph_on_participant_info(ctx);
            }
#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO && \
    !RMW_CONNEXT_USE_BUILTIN_LISTENER
            else if (DDS_StatusCondition_as_condition(cond_dcps_part) ==
                        cond_active)
            {
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] dcps-participants active")
                rmw_connextdds_discovery_thread_on_dcps_participant(ctx);
            }
            else if (DDS_StatusCondition_as_condition(cond_dcps_pub) ==
                        cond_active)
            {
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] dcps-publications active")
                rmw_connextdds_discovery_thread_on_dcps_publication(ctx);
            }
            else if (DDS_StatusCondition_as_condition(cond_dcps_sub) ==
                        cond_active)
            {
                RMW_CONNEXT_LOG_DEBUG(
                    "[discovery thread] dcps-subscriptions active")
                rmw_connextdds_discovery_thread_on_dcps_subscription(ctx);
            }
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */
            else
            {
                RMW_CONNEXT_LOG_ERROR("unexpected active condition")
                goto cleanup;
            }
        }
    }

    RMW_CONNEXT_LOG_DEBUG("[discovery thread] main loop terminated")

cleanup:

    RMW_CONNEXT_LOG_DEBUG("[discovery thread] cleaning up...")

    DDS_ConditionSeq_finalize(&active_conditions);

    if (nullptr != waitset)
    {
        if (attached_exit)
        {
            if (DDS_RETCODE_OK !=
                    DDS_WaitSet_detach_condition(waitset, cond_exit))
            {
                RMW_CONNEXT_LOG_ERROR(
                    "failed to detach graph condition from "
                    "discovery thread waitset")
                return;
            }
        }
        if (attached_partinfo)
        {
            if (DDS_RETCODE_OK !=
                    DDS_WaitSet_detach_condition(waitset, cond_partinfo))
            {
                RMW_CONNEXT_LOG_ERROR(
                    "failed to detach participant info condition from "
                    "discovery thread waitset")
                return;
            }
        }
#if RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO && \
    !RMW_CONNEXT_USE_BUILTIN_LISTENER
        if (attached_dcps_part)
        {
            if (DDS_RETCODE_OK !=
                    DDS_WaitSet_detach_condition(waitset,
                        DDS_StatusCondition_as_condition(cond_dcps_part)))
            {
                RMW_CONNEXT_LOG_ERROR(
                    "failed to detach DCPS Participant condition from "
                    "discovery thread waitset")
                return;
            }
        }
        if (attached_dcps_sub)
        {
            if (DDS_RETCODE_OK !=
                    DDS_WaitSet_detach_condition(waitset,
                        DDS_StatusCondition_as_condition(cond_dcps_sub)))
            {
                RMW_CONNEXT_LOG_ERROR(
                    "failed to detach DCPS Subscription condition from "
                    "discovery thread waitset")
                return;
            }
        }
        if (attached_dcps_pub)
        {
            if (DDS_RETCODE_OK !=
                    DDS_WaitSet_detach_condition(waitset,
                        DDS_StatusCondition_as_condition(cond_dcps_pub)))
            {
                RMW_CONNEXT_LOG_ERROR(
                    "failed to detach DCPS Publication condition from "
                    "discovery thread waitset")
                return;
            }
        }
#endif /* RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO */
        DDS_WaitSet_delete(waitset);
    }

    RMW_CONNEXT_LOG_DEBUG("[discovery thread] done")

    return;
}

rmw_ret_t
rmw_connextdds_discovery_thread_start(rmw_context_impl_t * ctx)
{
    rmw_dds_common::Context *const common_ctx = &ctx->common;

    RMW_CONNEXT_LOG_DEBUG("starting discovery thread...")

    common_ctx->listener_thread_gc =
        rmw_connextdds_create_guard_condition(true /* internal */);

    if (nullptr == common_ctx->listener_thread_gc)
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to create discovery thread condition")
        return RMW_RET_ERROR;
    }

    common_ctx->thread_is_running.store(true);

    try
    {
        common_ctx->listener_thread =
            std::thread(rmw_connextdds_discovery_thread, ctx);
        
        RMW_CONNEXT_LOG_DEBUG("discovery thread started")

        return RMW_RET_OK;
    }
    catch (const std::exception & exc)
    {
        RMW_CONNEXT_LOG_ERROR("Failed to create std::thread")
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Failed to create std::thread: %s", exc.what());
    }
    catch (...)
    {
        RMW_CONNEXT_LOG_ERROR("Failed to create std::thread")
    }

    /* We'll get here only on error, so clean up things accordingly */

    common_ctx->thread_is_running.store(false);
    if (RMW_RET_OK !=
            rmw_connextdds_destroy_guard_condition(
                common_ctx->listener_thread_gc))
    {
        RMW_CONNEXT_LOG_ERROR(
            "Failed to destroy discovery thread guard condition")
    }

    return RMW_RET_ERROR;
}

rmw_ret_t
rmw_connextdds_discovery_thread_stop(rmw_context_impl_t * ctx)
{
    rmw_dds_common::Context *const common_ctx = &ctx->common;

    RMW_CONNEXT_LOG_DEBUG("stopping discovery thread...")

    if (common_ctx->thread_is_running.exchange(false))
    {
        rmw_ret_t rmw_ret =
            rmw_trigger_guard_condition(common_ctx->listener_thread_gc);
        
        if (RMW_RET_OK != rmw_ret)
        {
            return rmw_ret;
        }

        try
        {
            common_ctx->listener_thread.join();
        }
        catch (const std::exception & exc)
        {
            RMW_CONNEXT_LOG_ERROR("Failed to join std::thread")
            RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
                "Failed to join std::thread: %s", exc.what());
            return RMW_RET_ERROR;
        }
        catch (...)
        {
            RMW_CONNEXT_LOG_ERROR("Failed to join std::thread")
            return RMW_RET_ERROR;
        }

        rmw_ret = rmw_connextdds_destroy_guard_condition(
                        common_ctx->listener_thread_gc);
        if (RMW_RET_OK != rmw_ret)
        {
            return rmw_ret;
        }
    }

    RMW_CONNEXT_LOG_DEBUG("discovery thread stopped")
    return RMW_RET_OK;
}

