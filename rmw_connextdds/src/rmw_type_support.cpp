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

#include <string.h>
#include <string>

#include "rmw_connextdds/type_support.hpp"

#include "rmw_connextdds/rmw_impl.hpp"


/******************************************************************************
 * RMW_Connext_MessageTypeSupport
 ******************************************************************************/

RMW_Connext_MessageTypeSupport::RMW_Connext_MessageTypeSupport(
    const RMW_Connext_MessageType message_type,
    const rosidl_message_type_support_t *const type_supports,
    const char *const type_name)
    : _type_support_fastrtps(
        RMW_Connext_MessageTypeSupport::get_type_support_fastrtps(
            type_supports)),
      _unbounded(false),
      _empty(false),
      _serialized_size_max(0),
      _type_name(),
      _message_type(message_type)
{
    auto callbacks = this->callbacks_fastrtps();

    switch (this->_message_type)
    {
    case RMW_CONNEXT_MESSAGE_USERDATA:
    {
        this->_type_name = rmw_connextdds_create_type_name(callbacks);
        break;
    }
    case RMW_CONNEXT_MESSAGE_REQUEST:
    case RMW_CONNEXT_MESSAGE_REPLY:
    {
        RMW_CONNEXT_ASSERT(type_name != nullptr)
        this->_type_name = type_name;
        break;
    }
    default:
        // should never get here
        break;
    }

    RMW_Connext_MessageTypeSupport::type_info(
        this->_type_support_fastrtps,
        this->_serialized_size_max,
        this->_unbounded,
        this->_empty);

#if RMW_CONNEXT_EMULATE_REQUESTREPLY
    if (!this->unbounded() && this->type_requestreply())
    {
        /* Add request header to the serialized buffer */
        this->_serialized_size_max += RMW_GID_STORAGE_SIZE + sizeof(int64_t);
    }
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */

    RMW_CONNEXT_LOG_DEBUG_A("[type support] new %s: "
        "type.unbounded=%d, "
        "type.empty=%d, "
        "type.serialized_size_max=%u, "
        "type.reqreply=%d",
        this->type_name(),
        this->_unbounded,
        this->_empty,
        this->_serialized_size_max,
        this->type_requestreply())
}

rmw_ret_t RMW_Connext_MessageTypeSupport::serialize(
    const void *const ros_msg,
    rcutils_uint8_array_t *const to_buffer)
{
    auto callbacks =
        static_cast<const message_type_support_callbacks_t *>(
            this->_type_support_fastrtps->data);

    eprosima::fastcdr::FastBuffer cdr_buffer(
        reinterpret_cast<char *>(to_buffer->buffer),
        to_buffer->buffer_capacity);
    eprosima::fastcdr::Cdr cdr_stream(
        cdr_buffer,
        eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        eprosima::fastcdr::Cdr::DDS_CDR);
    cdr_stream.setDDSCdrPlFlag(
        eprosima::fastcdr::Cdr::DDSCdrPlFlag::DDS_CDR_WITHOUT_PL);

    RMW_CONNEXT_LOG_DEBUG_A("[type support] %s serialize: "
        "type.unbounded=%d, "
        "type.empty=%d, "
        "type.serialized_size_max=%u, "
        "buffer.capacity=%lu, "
        "msg.serialized_size_max=%u",
        this->_type_name.c_str(),
        this->_unbounded,
        this->_empty,
        this->_serialized_size_max,
        to_buffer->buffer_capacity,
        this->serialized_size_max(ros_msg))

    const void *payload = ros_msg;

    try
    {
        cdr_stream.serialize_encapsulation();

        if (this->type_requestreply())
        {
            const RMW_Connext_RequestReplyMessage *const rr_msg =
                reinterpret_cast<const RMW_Connext_RequestReplyMessage*>(ros_msg);
            payload = rr_msg->payload;

#if RMW_CONNEXT_EMULATE_REQUESTREPLY
            /* Since Micro doesn't support sample_identity/related_sample_identity,
               we encode the request's sequence number and originator's guid as a
            header to the data payload */
            for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++)
            {
                cdr_stream << rr_msg->gid.data[i];
            }

            cdr_stream << rr_msg->sn;
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY  */
        }

        if (!this->_empty) {
            try
            {
                if (!callbacks->cdr_serialize(payload, cdr_stream))
                {
                    return RMW_RET_ERROR;
                }
            }
            catch (const std::exception & exc)
            {
                RMW_CONNEXT_LOG_ERROR("Failed to deserialize data")
                RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
                    "Failed to deserialize data: %s", exc.what());
                return RMW_RET_ERROR;
            }
            catch (...)
            {
                RMW_CONNEXT_LOG_ERROR("Failed to deserialize data")
                return RMW_RET_ERROR;
            }
        } else {
            /* Serialize a dummy byte */
            cdr_stream << (uint8_t)0;
        }
    }
    catch (const std::exception & exc)
    {
        RMW_CONNEXT_LOG_ERROR_A(
            "Failed to serialize ROS message: %s", exc.what())
        return RMW_RET_ERROR;
    }
    catch (...)
    {
        RMW_CONNEXT_LOG_ERROR("Failed to serialize ROS message")
        return RMW_RET_ERROR;
    }

    to_buffer->buffer_length =
        reinterpret_cast<uint8_t*>(cdr_stream.getCurrentPosition()) - to_buffer->buffer;

    RMW_CONNEXT_LOG_DEBUG_A("[type support] %s serialized: "
        "buffer.length=%lu",
        this->_type_name.c_str(),
        to_buffer->buffer_length)

    return RMW_RET_OK;
}

rmw_ret_t
RMW_Connext_MessageTypeSupport::deserialize(
    void *const ros_msg,
    const rcutils_uint8_array_t *const from_buffer,
    size_t &size_out)
{
    auto callbacks =
        static_cast<const message_type_support_callbacks_t *>(
            this->_type_support_fastrtps->data);

    eprosima::fastcdr::FastBuffer cdr_buffer(
        reinterpret_cast<char *>(from_buffer->buffer),
        from_buffer->buffer_length);
    eprosima::fastcdr::Cdr cdr_stream(
        cdr_buffer,
        eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        eprosima::fastcdr::Cdr::DDS_CDR);

    RMW_CONNEXT_LOG_DEBUG_A("[type support] %s deserialize: "
        "sample=%p, "
        "type.unbounded=%d, "
        "type.empty=%d, "
        "type.serialized_size_max=%u, "
        "buffer.length=%lu, "
        "buffer.capacity=%lu",
        this->type_name(),
        ros_msg,
        this->_unbounded,
        this->_empty,
        this->_serialized_size_max,
        from_buffer->buffer_length,
        from_buffer->buffer_capacity)

    void *payload = ros_msg;

    try
    {
        cdr_stream.read_encapsulation();

        if (this->type_requestreply())
        {
            RMW_Connext_RequestReplyMessage *const rr_msg =
                reinterpret_cast<RMW_Connext_RequestReplyMessage*>(ros_msg);

            payload = reinterpret_cast<void*>(rr_msg->payload);

#if RMW_CONNEXT_EMULATE_REQUESTREPLY
            for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++)
            {
                cdr_stream >> rr_msg->gid.data[i];
            }
            cdr_stream >> rr_msg->sn;
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */
        }

        if (!this->_empty) {
            try
            {
                if (!callbacks->cdr_deserialize(cdr_stream, payload))
                {
                    return RMW_RET_ERROR;
                }
            }
            catch (const std::exception & exc)
            {
                RMW_CONNEXT_LOG_ERROR("Failed to deserialize data")
                RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
                    "Failed to deserialize data: %s", exc.what());
                return RMW_RET_ERROR;
            }
            catch (...)
            {
                RMW_CONNEXT_LOG_ERROR("Failed to deserialize data")
                return RMW_RET_ERROR;
            }
        } else {
            /* Consume dummy byte */
            uint8_t dummy = 0;
            cdr_stream >> dummy;
            (void)dummy;
        }
    }
    catch (const std::exception & exc)
    {
        RMW_CONNEXT_LOG_ERROR_A(
            "Failed to deserialize ROS message: %s", exc.what())
        return RMW_RET_ERROR;
    }
    catch (...)
    {
        RMW_CONNEXT_LOG_ERROR("Failed to deserialize ROS message")
        return RMW_RET_ERROR;
    }

    size_out =
        cdr_stream.getCurrentPosition() -
            reinterpret_cast<char *>(from_buffer->buffer);

    RMW_CONNEXT_LOG_DEBUG_A("[type support] %s deserialized: "
        "consumed=%lu",
        this->_type_name.c_str(),
        size_out)

    return RMW_RET_OK;
}


uint32_t RMW_Connext_MessageTypeSupport::serialized_size_max(
    const void *const ros_msg,
    const bool include_encapsulation)
{
    /* Include size of encapsulation header in message's serialized size */
    uint32_t serialized_size = 0;

    if (include_encapsulation)
    {
        serialized_size +=
            RMW_Connext_MessageTypeSupport::ENCAPSULATION_HEADER_SIZE;
    }

    auto callbacks =
        static_cast<const message_type_support_callbacks_t *>(
            this->_type_support_fastrtps->data);


    if (!this->_unbounded) {
        serialized_size += this->_serialized_size_max;
    } else {
        const void *payload = ros_msg;
        if (this->type_requestreply())
        {
            const RMW_Connext_RequestReplyMessage * rr_msg =
                reinterpret_cast<const RMW_Connext_RequestReplyMessage*>(ros_msg);
            payload = rr_msg->payload;
        }
        serialized_size += callbacks->get_serialized_size(payload);
#if RMW_CONNEXT_EMULATE_REQUESTREPLY
        if (this->type_requestreply())
        {
            /* Add request header to serialized payload */
            serialized_size += RMW_GID_STORAGE_SIZE + sizeof(int64_t);
        }
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */
    }

    return serialized_size;
}

const rosidl_message_type_support_t*
RMW_Connext_MessageTypeSupport::get_type_support_fastrtps(
    const rosidl_message_type_support_t *const type_supports)
{
    const rosidl_message_type_support_t * type_support =
        get_message_typesupport_handle(
            type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_C);
    if (nullptr == type_support)
    {
        type_support = get_message_typesupport_handle(
        type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
    }
    return type_support;
}
#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
const rosidl_message_type_support_t*
RMW_Connext_MessageTypeSupport::get_type_support_intro(
    const rosidl_message_type_support_t *const type_supports,
    bool &cpp_version)
{
    const rosidl_message_type_support_t * type_support =
        get_message_typesupport_handle(
            type_supports, rosidl_typesupport_introspection_c__identifier);
    if (nullptr == type_support) {
        type_support =
            get_message_typesupport_handle(
                type_supports,
                rosidl_typesupport_introspection_cpp::typesupport_identifier);
        if (nullptr != type_support)
        {
            cpp_version = true;
        }
    } else {
        cpp_version = false;
    }
    return type_support;
}
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */

RMW_Connext_MessageTypeSupport *
RMW_Connext_MessageTypeSupport::register_type_support(
    rmw_context_impl_t *const ctx,
    const rosidl_message_type_support_t *const type_supports,
    DDS_DomainParticipant *const participant,
    bool& registered,
    const RMW_Connext_MessageType message_type,
    const void *const intro_members,
    const bool intro_members_cpp,
    std::string *const type_name)
{
    return rmw_connextdds_register_type_support(
                ctx,
                type_supports,
                participant,
                registered,
                message_type,
                intro_members,
                intro_members_cpp,
                (nullptr != type_name)?type_name->c_str():nullptr);
}

rmw_ret_t
RMW_Connext_MessageTypeSupport::unregister_type_support(
    rmw_context_impl_t *const ctx,
    DDS_DomainParticipant *const participant,
    const char *const type_name)
{
    return rmw_connextdds_unregister_type_support(
                ctx, participant, type_name);
}

void RMW_Connext_MessageTypeSupport::type_info(
    const rosidl_message_type_support_t *const type_support,
    uint32_t &serialized_size_max,
    bool &unbounded,
    bool &empty)
{
    serialized_size_max = 0;
    unbounded = false;
    empty = false;

    auto callbacks =
        static_cast<const message_type_support_callbacks_t *>(type_support->data);

    /* The fastrtps type support sets full_bounded to false if unbounded,
       but assumes full_bounded == true by default */
    bool full_bounded = true;

    serialized_size_max =
        static_cast<uint32_t>(callbacks->max_serialized_size(full_bounded));

    unbounded = !full_bounded;

    if (unbounded && serialized_size_max == 0)
    {
        /* Empty message */
        empty = true;
        serialized_size_max = 1;
    }

    /* add encapsulation size to static serialized_size_max */
    serialized_size_max +=
        RMW_Connext_MessageTypeSupport::ENCAPSULATION_HEADER_SIZE;
}


/******************************************************************************
 * Service Type Support
 ******************************************************************************/
#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
const rosidl_service_type_support_t *
RMW_Connext_ServiceTypeSupportWrapper::get_type_support_intro(
    const rosidl_service_type_support_t *const type_supports,
    bool &cpp_version)
{
    cpp_version = false;
    const rosidl_service_type_support_t *type_support =
        get_service_typesupport_handle(
            type_supports, rosidl_typesupport_introspection_c__identifier);

    if (nullptr == type_support)
    {
        type_support =
            get_service_typesupport_handle(
                type_supports,
                rosidl_typesupport_introspection_cpp::typesupport_identifier);
        if (nullptr != type_support)
        {
            cpp_version = true;
        }
    }

    return type_support;
}
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */

const rosidl_service_type_support_t *
RMW_Connext_ServiceTypeSupportWrapper::get_type_support_fastrtps(
    const rosidl_service_type_support_t *const type_supports)
{
    const rosidl_service_type_support_t *type_support =
        get_service_typesupport_handle(
            type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_C);

    if (nullptr == type_support)
    {
        type_support = get_service_typesupport_handle(
            type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
    }

    return type_support;
}

std::string
RMW_Connext_ServiceTypeSupportWrapper::get_request_type_name(
    const rosidl_service_type_support_t *const type_supports)
{
    const rosidl_service_type_support_t *const svc_type_support_fastrtps =
        RMW_Connext_ServiceTypeSupportWrapper::get_type_support_fastrtps(
            type_supports);
    if (nullptr == svc_type_support_fastrtps)
    {
        std::string empty_name;
        return empty_name;
    }
    auto svc_callbacks =
        static_cast<const service_type_support_callbacks_t *>(
            svc_type_support_fastrtps->data);

    return rmw_connextdds_create_type_name_request(svc_callbacks);
}

std::string
RMW_Connext_ServiceTypeSupportWrapper::get_response_type_name(
    const rosidl_service_type_support_t *const type_supports)
{
    const rosidl_service_type_support_t *const svc_type_support_fastrtps =
        RMW_Connext_ServiceTypeSupportWrapper::get_type_support_fastrtps(
            type_supports);
    if (nullptr == svc_type_support_fastrtps)
    {
        std::string empty_name;
        return empty_name;
    }
    auto svc_callbacks =
        static_cast<const service_type_support_callbacks_t *>(
            svc_type_support_fastrtps->data);

    return rmw_connextdds_create_type_name_response(svc_callbacks);
}
