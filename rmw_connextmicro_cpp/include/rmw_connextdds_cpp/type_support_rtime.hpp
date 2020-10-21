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

#ifndef RMW_CONNEXT__TYPE_SUPPORT_RTIME_HPP_
#define RMW_CONNEXT__TYPE_SUPPORT_RTIME_HPP_

struct RMW_Connext_RtimeTypePlugin
{
    static
    RMW_Connext_MessageTypeSupport*
    register_type_support(
        rmw_context_impl_t *const ctx,
        const rosidl_message_type_support_t *const type_supports,
        DDS_DomainParticipant *const participant,
        bool& registered,
        const RMW_Connext_MessageType message_type,
        const void *const intro_members = nullptr,
        const bool intro_members_cpp = false,
        std::string *const type_name = nullptr);
    
    static
    rmw_ret_t
    unregister_type_support(
        rmw_context_impl_t *const ctx,
        DDS_DomainParticipant *const participant,
        const char *const type_name);
};

typedef struct RMW_Connext_RtimeTypePlugin
        RMW_Connext_TypePlugin;
        
typedef DDS_UntypedSampleSeq RMW_Connext_UntypedSampleSeq;

#define RMW_Connext_UntypedSampleSeq_INITIALIZER    DDS_SEQUENCE_INITIALIZER

#endif /* RMW_CONNEXT__TYPE_SUPPORT_HPP_ */