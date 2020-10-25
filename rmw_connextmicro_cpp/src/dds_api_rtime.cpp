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

#include "rmw_connextdds_cpp/type_support.hpp"
#include "rmw_connextdds_cpp/rmw_impl.hpp"

#include "rcutils/get_env.h"

struct rmw_connextdds_api_micro
{
    bool rt_whsm;
    bool rt_rhsm;
    bool rt_udp;
    bool rt_shmem;
    bool rt_dpde;
    bool rt_sec;
    struct UDP_InterfaceFactoryProperty *udp_property;

    rmw_connextdds_api_micro()
    : rt_whsm(false),
      rt_rhsm(false),
      rt_udp(false),
      rt_shmem(false),
      rt_dpde(false),
      rt_sec(false),
      udp_property(nullptr)
    {

    }
};

const char * const RMW_CONNEXTDDS_ID = "rmw_connextmicro_cpp";
const char * const RMW_CONNEXTDDS_SERIALIZATION_FORMAT = "cdr";

rmw_ret_t
rmw_connextdds_set_log_verbosity(rmw_log_severity_t severity)
{
    OSAPI_LogVerbosity_T verbosity = OSAPI_LOG_VERBOSITY_ERROR;

    switch (severity)
    {
    case RMW_LOG_SEVERITY_DEBUG:
    case RMW_LOG_SEVERITY_INFO:
    {
        verbosity = OSAPI_LOG_VERBOSITY_DEBUG;
        break;
    }
    case RMW_LOG_SEVERITY_WARN:
    {
        verbosity = OSAPI_LOG_VERBOSITY_WARNING;
        break;
    }
    case RMW_LOG_SEVERITY_ERROR:
    case RMW_LOG_SEVERITY_FATAL:
    {
        verbosity = OSAPI_LOG_VERBOSITY_ERROR;
        break;
    }
    default:
    {
        RMW_CONNEXT_LOG_ERROR("invalid log level")
        return RMW_RET_INVALID_ARGUMENT;
    }
    }

    OSAPI_Log_set_verbosity(verbosity);
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_initialize_participant_factory(
    rmw_context_impl_t *const ctx)
{
    DDS_DomainParticipantFactory *factory = NULL;
    RT_Registry_T *registry = NULL;
    
    struct OSAPI_LogProperty log_prop;

    const char *const whsm_name = DDSHST_WRITER_DEFAULT_HISTORY_NAME,
               *const rhsm_name = DDSHST_READER_DEFAULT_HISTORY_NAME,
               *const udp_name = NETIO_DEFAULT_UDP_NAME,
               *const shmem_name = NETIO_DEFAULT_SHMEM_NAME,
               *const dpde_name = "dpde";

    rmw_connextdds_api_micro *ctx_api = nullptr;

    auto scope_exit_api_delete = rcpputils::make_scope_exit(
        [ctx_api, registry, whsm_name, rhsm_name, udp_name, shmem_name, dpde_name]()
        {
            if (ctx_api->rt_whsm)
            {
                if(!RT_Registry_unregister(registry, whsm_name, NULL, NULL))
                {
                    RMW_CONNEXT_LOG_ERROR("failed to unregister whsm")
                }
            }
            if (ctx_api->rt_rhsm)
            {
                if(!RT_Registry_unregister(registry, rhsm_name, NULL, NULL))
                {
                    RMW_CONNEXT_LOG_ERROR("failed to unregister rhsm")
                }
            }
            if (ctx_api->rt_udp)
            {
                if(!RT_Registry_unregister(registry, udp_name, NULL, NULL))
                {
                    RMW_CONNEXT_LOG_ERROR("failed to unregister udp")
                }
            }
            if (ctx_api->rt_dpde)
            {
                if(!RT_Registry_unregister(registry, dpde_name, NULL, NULL))
                {
                    RMW_CONNEXT_LOG_ERROR("failed to unregister dpde")
                }
            }
            if (nullptr != ctx_api->udp_property)
            {
                DDS_StringSeq_finalize(&ctx_api->udp_property->allow_interface);
                delete ctx_api->udp_property;
            }
            delete ctx_api;
            DDS_DomainParticipantFactory_finalize_instance();
        });

        RMW_CONNEXT_LOG_DEBUG("initializing DDS DomainParticipantFactory")

    OSAPI_Log_get_property(&log_prop);
    log_prop.max_buffer_size = OSAPI_LOG_BUFFER_SIZE *100;
    OSAPI_Log_set_property(&log_prop);

    factory = DDS_DomainParticipantFactory_get_instance();
    if (nullptr == factory)
    {
        RMW_CONNEXT_LOG_ERROR("failed to get DDS participant factory")
        return RMW_RET_ERROR;
    }

    ctx_api = new (std::nothrow) rmw_connextdds_api_micro();
    if (nullptr == ctx_api)
    {
        return RMW_RET_ERROR;
    }

    registry = DDS_DomainParticipantFactory_get_registry(factory);
    if (nullptr == registry)
    {
        RMW_CONNEXT_LOG_ERROR("failed to get factory registry")
        return RMW_RET_ERROR;
    }

    /* Configure Writer and Reader History plugins */

    if (!RT_Registry_register(registry,
            whsm_name, WHSM_HistoryFactory_get_interface(), NULL, NULL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to register Writer History plugin")
        return RMW_RET_ERROR;
    }

    ctx_api->rt_whsm = true;

    if (!RT_Registry_register(registry,
            rhsm_name, RHSM_HistoryFactory_get_interface(), NULL, NULL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to register Reader History plugin")
        return RMW_RET_ERROR;
    }

    ctx_api->rt_rhsm = true;

    /* Configure UDP Transport plugin */

    if(!RT_Registry_unregister(registry, udp_name, NULL, NULL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to unregister udp")
        return RMW_RET_ERROR;
    }

    ctx_api->udp_property =  new (std::nothrow) UDP_InterfaceFactoryProperty();
    
    if (nullptr == ctx_api->udp_property)
    {
        RMW_CONNEXT_LOG_ERROR("failed to allocate UDP Transport property")
        return RMW_RET_ERROR;
    }

    *ctx_api->udp_property = UDP_INTERFACE_FACTORY_PROPERTY_DEFAULT;

    if (!DDS_StringSeq_set_maximum(&ctx_api->udp_property->allow_interface, 1))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set allow_interface maximum")
        return RMW_RET_ERROR;
    }
    if (!DDS_StringSeq_set_length(&ctx_api->udp_property->allow_interface, 1))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set allow_interface length")
        return RMW_RET_ERROR;
    }

    /* Lookup name of UDP interface from environment */
    const char *env_udp_intf = nullptr;
    const char *lookup_rc =
        rcutils_get_env(RMW_CONNEXT_ENV_UDP_INTERFACE, &env_udp_intf);
    
    if (nullptr != lookup_rc || nullptr == env_udp_intf)
    {
        RMW_CONNEXT_LOG_ERROR_A("failed to lookup from environment: "
            "var=%s, "
            "rc=%s ",
            RMW_CONNEXT_ENV_UDP_INTERFACE,
            lookup_rc)
        return RMW_RET_ERROR;
    }

    if (strlen(env_udp_intf) == 0)
    {
        env_udp_intf = RMW_CONNEXT_DEFAULT_UDP_INTERFACE;
    }

    char *udp_intf = DDS_String_dup(env_udp_intf);
    if (nullptr == udp_intf)
    {
        RMW_CONNEXT_LOG_ERROR("failed to allocate udp interface name")
        return RMW_RET_ERROR;
    }

    *DDS_StringSeq_get_reference(&ctx_api->udp_property->allow_interface, 0) = 
            udp_intf;
    
    RMW_CONNEXT_LOG_DEBUG_A("UDP interface: %s", udp_intf)

    if (!RT_Registry_register(registry,
            udp_name,
            UDP_InterfaceFactory_get_interface(),
            (struct RT_ComponentFactoryProperty*)ctx_api->udp_property, NULL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to register UDP Transport plugin")
        return RMW_RET_ERROR;
    }

    ctx_api->rt_udp = true;

    struct NETIO_SHMEMInterfaceFactoryProperty shmem_property =
        NETIO_SHMEMInterfaceFactoryProperty_INITIALIZER;

    if (!RT_Registry_register(registry,
            shmem_name,
            NETIO_SHMEMInterfaceFactory_get_interface(),
            (struct RT_ComponentFactoryProperty*)&shmem_property, NULL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to register SHMEM Transport plugin")
        return RMW_RET_ERROR;
    }

    ctx_api->rt_shmem = true;

    struct DPDE_DiscoveryPluginProperty discovery_plugin_properties =
                DPDE_DiscoveryPluginProperty_INITIALIZER;

#if defined(CONFIGURE_DPDE_LIVELINESS_SOMEHOW) || 1
    discovery_plugin_properties.participant_liveliness_assert_period.sec =
            10;
    discovery_plugin_properties.participant_liveliness_assert_period.nanosec =
            0;

    discovery_plugin_properties.participant_liveliness_lease_duration.sec =
            60;
    discovery_plugin_properties.participant_liveliness_lease_duration.nanosec =
            0;
#endif

    if (!RT_Registry_register(
            registry,
            dpde_name,
            DPDE_DiscoveryFactory_get_interface(),
            &discovery_plugin_properties._parent,
            NULL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to register DPDE Discovery plugin")
        return RMW_RET_ERROR;
    }

    ctx_api->rt_dpde = true;


#if defined(CONFIGURE_SECURITY_SOMEHOW)
    /* Configure DDS Security plugins */
    {
        struct SECCORE_SecurePluginFactoryProperty sec_plugin_prop =
                SECCORE_SecurePluginFactoryProperty_INITIALIZER;

        if (!SECCORE_SecurePluginFactory_register(
                    registry, SECCORE_DEFAULT_SUITE_NAME, &sec_plugin_prop))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to register DDS Security plugins")
            return RMW_RET_ERROR;
        }
    }
#endif
    /* We do this here instead of rmw_context_impl_t::initialize_node
       because we can clean up the participant factory upon error */
    if (RMW_RET_OK !=
            rmw_connextdds_initialize_participant_factory_qos(ctx, factory))
    {
        RMW_CONNEXT_LOG_ERROR(
            "failed to initialize DDS DomainParticipantFactory QoS")
        return RMW_RET_ERROR;
    }

    scope_exit_api_delete.cancel();

    RMW_CONNEXT_LOG_DEBUG("DDS DomainParticipantFactory initialized")

    ctx->factory = factory;
    ctx->api = ctx_api;

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_finalize_participant_factory(
    rmw_context_impl_t *const ctx)
{
    rmw_connextdds_api_micro *const ctx_api =
        (rmw_connextdds_api_micro*)ctx->api;

    RT_Registry_T *registry =
            DDS_DomainParticipantFactory_get_registry(ctx->factory);
    if (nullptr == registry)
    {
        RMW_CONNEXT_LOG_ERROR("failed to get factory registry")
        return RMW_RET_ERROR;
    }

    if (ctx_api->rt_whsm)
    {
        if (!RT_Registry_unregister(
                registry,
                DDSHST_WRITER_DEFAULT_HISTORY_NAME,
                NULL, NULL))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to unregister Writer History plugin")
            return RMW_RET_ERROR;
        }
        ctx_api->rt_whsm = false;
    }

    if (ctx_api->rt_rhsm)
    {
        if (!RT_Registry_unregister(
                registry,
                DDSHST_READER_DEFAULT_HISTORY_NAME,
                NULL, NULL))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to unregister Reader History plugin")
            return RMW_RET_ERROR;
        }
        ctx_api->rt_rhsm = false;
    }

    if (ctx_api->rt_udp)
    {
        if (!RT_Registry_unregister(
                registry,
                NETIO_DEFAULT_UDP_NAME,
                NULL, NULL))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to unregister UDP Transport plugin")
            return RMW_RET_ERROR;
        }
        ctx_api->rt_udp = false;
    }

    if (ctx_api->rt_shmem)
    {
        if (!RT_Registry_unregister(
                registry,
                NETIO_DEFAULT_SHMEM_NAME,
                NULL, NULL))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to unregister SHMEM Transport plugin")
            return RMW_RET_ERROR;
        }
        ctx_api->rt_shmem = false;
    }

    if (ctx_api->rt_dpde)
    {
        if (!RT_Registry_unregister(registry, "dpde", NULL, NULL))
        {
            RMW_CONNEXT_LOG_ERROR(
                "failed to unregister DPDE Discovery plugin")
            return RMW_RET_ERROR;
        }
        ctx_api->rt_rhsm = false;
    }
    
    if (nullptr != ctx_api->udp_property)
    {
        UDP_InterfaceFactoryProperty_finalize(ctx_api->udp_property);
        delete ctx_api->udp_property;
        ctx_api->udp_property = nullptr;
    }

    delete ctx_api;

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_initialize_participant_qos_impl(
    rmw_context_impl_t *const ctx,
    DDS_DomainParticipantQos *const dp_qos)
{
    UNUSED_ARG(ctx);
    
    /* Store enclave's name in USER_DATA field */
    // std::string user_data = std::string("enclave=") + std::string(
    //     options->enclave) + std::string(";");

    /* TODO Micro doesn't support USER DATA */
    
    /* Configure DDS Security options */
    // if (configure_qos_for_security(
    //     ppant_qos.get(),
    //     &options->security_options) != RMW_RET_OK)
    // {
    //     if (RMW_SECURITY_ENFORCEMENT_ENFORCE == options->security_options.enforce_security) {
    //     return RMW_RET_ERROR;
    //     }
    // }

    /* Mark only UDP transport as enabled, and skip INTRA */
    if (!REDA_StringSeq_set_maximum(&dp_qos->transports.enabled_transports, 2))
    {
        return RMW_RET_ERROR;
    }
    if (!REDA_StringSeq_set_length(&dp_qos->transports.enabled_transports, 2))
    {
        return RMW_RET_ERROR;
    }
    char **seq_ref = 
        REDA_StringSeq_get_reference(&dp_qos->transports.enabled_transports, 0);
    *seq_ref = REDA_String_dup(NETIO_DEFAULT_UDP_NAME);
    if (nullptr == *seq_ref)
    {
        return RMW_RET_ERROR;
    }

    seq_ref = 
        REDA_StringSeq_get_reference(&dp_qos->transports.enabled_transports, 1);
    *seq_ref = REDA_String_dup(NETIO_DEFAULT_SHMEM_NAME);
    if (nullptr == *seq_ref)
    {
        return RMW_RET_ERROR;
    }

    if (!REDA_StringSeq_set_maximum(&dp_qos->user_traffic.enabled_transports, 2))
    {
        return RMW_RET_ERROR;
    }
    if (!REDA_StringSeq_set_length(&dp_qos->user_traffic.enabled_transports, 2))
    {
        return RMW_RET_ERROR;
    }

    seq_ref = 
        REDA_StringSeq_get_reference(
            &dp_qos->user_traffic.enabled_transports, 0);
    std::ostringstream shmem_ss;
    shmem_ss << NETIO_DEFAULT_SHMEM_NAME << "://";
    
    *seq_ref = REDA_String_dup(shmem_ss.str().c_str());
    if (nullptr == *seq_ref)
    {
        return RMW_RET_ERROR;
    }

    seq_ref = 
        REDA_StringSeq_get_reference(
            &dp_qos->user_traffic.enabled_transports, 1);
    
    std::ostringstream udp_ss;
    udp_ss << NETIO_DEFAULT_UDP_NAME << "://";
    
    *seq_ref = REDA_String_dup(udp_ss.str().c_str());
    if (nullptr == *seq_ref)
    {
        return RMW_RET_ERROR;
    }

    /* Increate Participant resource limits */
    dp_qos->resource_limits.local_type_allocation =
        RMW_CONNEXT_LIMIT_TYPES_LOCAL_MAX;
    
    dp_qos->resource_limits.local_topic_allocation =
        RMW_CONNEXT_LIMIT_TOPICS_LOCAL_MAX;
    
    dp_qos->resource_limits.local_reader_allocation =
        RMW_CONNEXT_LIMIT_READERS_LOCAL_MAX;
    
    dp_qos->resource_limits.local_writer_allocation =
        RMW_CONNEXT_LIMIT_WRITERS_LOCAL_MAX;

    dp_qos->resource_limits.remote_participant_allocation =
        RMW_CONNEXT_LIMIT_PARTICIPANTS_REMOTE_MAX;
    
    dp_qos->resource_limits.remote_writer_allocation =
        RMW_CONNEXT_LIMIT_WRITERS_REMOTE_MAX;
    
    dp_qos->resource_limits.remote_reader_allocation =
        RMW_CONNEXT_LIMIT_READERS_REMOTE_MAX;
    
    dp_qos->resource_limits.local_publisher_allocation = 1;

    dp_qos->resource_limits.local_subscriber_allocation = 1;
    
    dp_qos->resource_limits.matching_reader_writer_pair_allocation =
        dp_qos->resource_limits.local_reader_allocation *
        dp_qos->resource_limits.remote_writer_allocation *
        dp_qos->resource_limits.remote_participant_allocation;
    
    dp_qos->resource_limits.matching_writer_reader_pair_allocation =
        dp_qos->resource_limits.local_writer_allocation *
        dp_qos->resource_limits.remote_reader_allocation *
        dp_qos->resource_limits.remote_participant_allocation;
    
    if (!RT_ComponentFactoryId_set_name(
            &dp_qos->discovery.discovery.name, "dpde"))
    {
        RMW_CONNEXT_LOG_ERROR("failed to set discovery plugin name")
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

static
rmw_ret_t
rmw_connextdds_get_qos_policies(
    const bool writer_qos,
    RMW_Connext_MessageTypeSupport *const type_support,
    DDS_HistoryQosPolicy *const history,
    DDS_ReliabilityQosPolicy *const reliability,
    DDS_DurabilityQosPolicy *const durability,
    DDS_DeadlineQosPolicy *const deadline,
    DDS_LivelinessQosPolicy *const liveliness,
    DDS_ResourceLimitsQosPolicy *const resource_limits,
    DDS_DataReaderResourceLimitsQosPolicy *const reader_resource_limits,
    DDS_DataWriterResourceLimitsQosPolicy *const writer_resource_limits,
    DDS_DataReaderProtocolQosPolicy *const reader_protocol,
    DDS_DataWriterProtocolQosPolicy *const writer_protocol,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_publisher_options_t *const pub_options,
    const rmw_subscription_options_t *const sub_options)
{
    UNUSED_ARG(type_support);
    UNUSED_ARG(writer_qos);
    UNUSED_ARG(reliability);
    UNUSED_ARG(durability);
    UNUSED_ARG(deadline);
    UNUSED_ARG(liveliness);
    UNUSED_ARG(qos_policies);
    UNUSED_ARG(pub_options);
    UNUSED_ARG(sub_options);

    /* Adjust resource limits according to other QoS policies */
    size_t max_samples = RMW_CONNEXT_LIMIT_SAMPLES_MAX;

    if (DDS_LENGTH_UNLIMITED != history->depth &&
        (size_t)history->depth > max_samples)
    {
        max_samples = history->depth;
    }
    if (max_samples < RMW_CONNEXT_LIMIT_SAMPLES_MAX)
    {
        max_samples = RMW_CONNEXT_LIMIT_SAMPLES_MAX;
    }

    resource_limits->max_samples_per_instance = max_samples;
    resource_limits->max_samples = max_samples;
    resource_limits->max_instances = 1; /* ROS doesn't use instances */

    RMW_CONNEXT_LOG_DEBUG_A("endpoint resource limits: "
        "max_samples_per_instance=%d, "
        "max_samples=%d, "
        "max_instances=%d",
        resource_limits->max_samples_per_instance,
        resource_limits->max_samples,
        resource_limits->max_instances);
    
    if (nullptr != reader_resource_limits)
    {
        reader_resource_limits->max_remote_writers = 
            RMW_CONNEXT_LIMIT_WRITERS_LOCAL_MAX +
            RMW_CONNEXT_LIMIT_WRITERS_REMOTE_MAX;
        reader_resource_limits->max_remote_writers_per_instance =
            RMW_CONNEXT_LIMIT_WRITERS_LOCAL_MAX +
            RMW_CONNEXT_LIMIT_WRITERS_REMOTE_MAX;
        // reader_resource_limits->max_samples_per_remote_writer = 0;
        reader_resource_limits->max_outstanding_reads = 1;
        reader_resource_limits->max_routes_per_writer = 1;
    }

    if (nullptr != writer_resource_limits)
    {
        writer_resource_limits->max_remote_readers =
            RMW_CONNEXT_LIMIT_READERS_LOCAL_MAX +
            RMW_CONNEXT_LIMIT_READERS_REMOTE_MAX;
        writer_resource_limits->max_routes_per_reader = 1;
    }

    if (nullptr != reader_protocol)
    {
        reader_protocol->rtps_reliable_reader.nack_period.sec = 0;
        reader_protocol->rtps_reliable_reader.nack_period.nanosec = 10000000;
    }

    if (nullptr != writer_protocol)
    {
        RMW_CONNEXT_ASSERT(nullptr != writer_resource_limits)
        writer_protocol->rtps_reliable_writer.heartbeats_per_max_samples =
            resource_limits->max_samples;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_get_datawriter_qos(
    rmw_context_impl_t *const ctx,
    RMW_Connext_MessageTypeSupport *const type_support,
    DDS_DataWriterQos *const qos,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_publisher_options_t *const pub_options)
{
    UNUSED_ARG(ctx);

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
                &qos->publish_mode,
                qos_policies,
                pub_options,
                nullptr /* sub_options */))
    {
        return RMW_RET_ERROR;
    }

    return rmw_connextdds_get_qos_policies(
                true /* writer_qos */,
                type_support,
                &qos->history,
                &qos->reliability,
                &qos->durability,
                &qos->deadline,
                &qos->liveliness,
                &qos->resource_limits,
                nullptr /* reader_resource_limits */,
                &qos->writer_resource_limits,
                nullptr /* reader protocol */,
                &qos->protocol,
                qos_policies,
                pub_options,
                nullptr /* sub_options */);
}

rmw_ret_t
rmw_connextdds_get_datareader_qos(
    rmw_context_impl_t *const ctx,
    RMW_Connext_MessageTypeSupport *const type_support,
    DDS_DataReaderQos *const qos,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_subscription_options_t *const sub_options)
{
    UNUSED_ARG(ctx);

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
                qos_policies,
                nullptr /* pub_options */,
                sub_options))
    {
        return RMW_RET_ERROR;
    }
    return rmw_connextdds_get_qos_policies(
                false /* writer_qos */,
                type_support,
                &qos->history,
                &qos->reliability,
                &qos->durability,
                &qos->deadline,
                &qos->liveliness,
                &qos->resource_limits,
                &qos->reader_resource_limits,
                nullptr /* writer_resource_limits */,
                &qos->protocol,
                nullptr /* writer protocol */,
                qos_policies,
                nullptr /* pub_options */,
                sub_options);
}

DDS_DataWriter*
rmw_connextdds_create_datawriter(
    rmw_context_impl_t *const ctx,
    DDS_DomainParticipant *const participant,
    DDS_Publisher *const pub,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_publisher_options_t *const publisher_options,
    const bool internal,
    RMW_Connext_MessageTypeSupport *const type_support,
    DDS_Topic *const topic,
    DDS_DataWriterQos *const dw_qos)
{
    UNUSED_ARG(ctx);
    UNUSED_ARG(participant);
    UNUSED_ARG(internal);

    if (RMW_RET_OK !=
            rmw_connextdds_get_datawriter_qos(
                ctx, type_support, dw_qos, qos_policies, publisher_options))
    {
        RMW_CONNEXT_LOG_ERROR("failed to convert writer QoS")
        return nullptr;
    }

    DDS_DataWriter *const writer =
            DDS_Publisher_create_datawriter(
                pub, topic, dw_qos, NULL, DDS_STATUS_MASK_NONE);
    
    return writer;
}

DDS_DataReader*
rmw_connextdds_create_datareader(
    rmw_context_impl_t *const ctx,
    DDS_DomainParticipant *const participant,
    DDS_Subscriber *const sub,
    const rmw_qos_profile_t *const qos_policies,
    const rmw_subscription_options_t *const subscriber_options,
    const bool internal,
    RMW_Connext_MessageTypeSupport *const type_support,
    DDS_Topic *const topic,
    DDS_DataReaderQos *const dr_qos)
{
    UNUSED_ARG(ctx);
    UNUSED_ARG(participant);
    UNUSED_ARG(internal);

    if (RMW_RET_OK !=
            rmw_connextdds_get_datareader_qos(
                ctx, type_support, dr_qos, qos_policies, subscriber_options))
    {
        RMW_CONNEXT_LOG_ERROR("failed to convert reader QoS")
        return nullptr;
    }

    return DDS_Subscriber_create_datareader(
                sub,
                DDS_Topic_as_topicdescription(topic),
                dr_qos,
                NULL, DDS_STATUS_MASK_NONE);
}

rmw_ret_t
rmw_connextdds_write_message(
    RMW_Connext_Publisher *const pub,
    RMW_Connext_Message *const message)
{
    if (DDS_RETCODE_OK !=
            DDS_DataWriter_write(pub->writer(), message, &DDS_HANDLE_NIL))
    {
        RMW_CONNEXT_LOG_ERROR("failed to write message to DDS")
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_take_samples(
    RMW_Connext_Subscriber *const sub)
{
    DDS_ReturnCode_t rc = 
        DDS_DataReader_take(
            sub->reader(),
            sub->data_seq(),
            sub->info_seq(),
            DDS_LENGTH_UNLIMITED,
            DDS_ANY_VIEW_STATE,
            DDS_ANY_SAMPLE_STATE,
            DDS_ANY_INSTANCE_STATE);

    if (DDS_RETCODE_NO_DATA == rc)
    {
        return RMW_RET_OK;
    }
    else if (DDS_RETCODE_OK != rc)
    {
        RMW_CONNEXT_LOG_ERROR("failed to take data from DDS reader")
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_return_samples(
    RMW_Connext_Subscriber *const sub)
{
    if (DDS_RETCODE_OK !=
            DDS_DataReader_return_loan(
                sub->reader(), sub->data_seq(), sub->info_seq()))
    {
        RMW_CONNEXT_LOG_ERROR("failed to return data to DDS reader")
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_filter_sample(
    RMW_Connext_Subscriber *const sub,
    const void *const sample,
    const DDS_SampleInfo *const info,
    bool *const accepted)
{
    UNUSED_ARG(sub);
    UNUSED_ARG(sample);
    UNUSED_ARG(info);
    *accepted = true;
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_participant_get_reader(
    rmw_context_impl_t *const ctx,
    DDS_DataReader **const reader_out)
{
    UNUSED_ARG(ctx);
    *reader_out = nullptr;
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_publication_get_reader(
    rmw_context_impl_t *const ctx,
    DDS_DataReader **const reader_out)
{
    UNUSED_ARG(ctx);
    *reader_out = nullptr;
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_subscription_get_reader(
    rmw_context_impl_t *const ctx,
    DDS_DataReader **const reader_out)
{
    UNUSED_ARG(ctx);
    *reader_out = nullptr;
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_enable_builtin_readers(rmw_context_impl_t *const ctx)
{
    UNUSED_ARG(ctx);
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_participant_on_data(rmw_context_impl_t *const ctx)
{
    UNUSED_ARG(ctx);
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_publication_on_data(rmw_context_impl_t *const ctx)
{
    UNUSED_ARG(ctx);
    return RMW_RET_OK;
}

rmw_ret_t
rmw_connextdds_dcps_subscription_on_data(rmw_context_impl_t *const ctx)
{
    UNUSED_ARG(ctx);
    return RMW_RET_OK;
}

void
rmw_connextdds_ih_to_gid(
    const DDS_InstanceHandle_t &ih, rmw_gid_t &gid)
{
    RTPS_Guid guid = RTPS_GUID_UNKNOWN;
    DDS_InstanceHandle_to_rtps(&guid, &ih);

    static_assert(
        RMW_GID_STORAGE_SIZE >= sizeof(guid),
        "rmw_gid_t type too small for an RTI Connext DDS Micro GUID");
    
    memset(&gid, 0, sizeof(gid));
    gid.implementation_identifier = RMW_CONNEXTDDS_ID;
    gid.data[0] = guid.prefix.host_id;
    gid.data[1] = guid.prefix.app_id ;
    gid.data[2] = guid.prefix.instance_id ;
    gid.data[3] = guid.object_id;
}

static
DDS_Boolean
RMW_Connext_DataReaderListener_before_sample_commit_drop_local(
    void *listener_data,
    DDS_DataReader *reader,
    const void *const sample,
    const struct DDS_SampleInfo *const sample_info,
    DDS_Boolean *dropped)
{
    UNUSED_ARG(reader);
    UNUSED_ARG(sample);

    RMW_Connext_StdSubscriberStatusCondition *const self =
        (RMW_Connext_StdSubscriberStatusCondition*)listener_data;
    
    *dropped = memcmp(
                self->drop_handle().octet,
                sample_info->publication_handle.octet,
                12) == 0 ?
                    DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;

    return DDS_BOOLEAN_TRUE;
}

void
rmw_connextdds_configure_subscriber_condition_listener(
    RMW_Connext_Subscriber *const sub,
    RMW_Connext_StdSubscriberStatusCondition *cond,
    DDS_DataReaderListener *const listener,
    DDS_StatusMask *const listener_mask)
{
    UNUSED_ARG(sub);
    UNUSED_ARG(listener_mask);
    if (sub->ignore_local())
    {
        listener->on_before_sample_commit =
            RMW_Connext_DataReaderListener_before_sample_commit_drop_local;
        cond->set_drop_handle(sub->instance_handle());
    }
}


void
rmw_connextdds_builtinkey_to_guid(
    const DDS_BuiltinTopicKey_t *const self,
    DDS_GUID_t *const dst)
{
    RTPS_Guid guid = RTPS_GUID_UNKNOWN;

    guid.prefix.host_id = self->value[0];
    guid.prefix.app_id = self->value[1];
    guid.prefix.instance_id = self->value[2];
    guid.object_id = self->value[3];

    DDS_GUID_from_rtps(dst, &guid);
}