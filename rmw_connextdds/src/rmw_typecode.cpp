// (c) 2020 Copyright, Real-Time Innovations, Inc. (RTI)
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

#include <string>

#include "rmw_connextdds/typecode.hpp"
#include "rmw_connextdds/rmw_impl.hpp"

static RTIBool
RMW_Connext_TypeCodePtr_initialize_w_params(
    DDS_TypeCode **self,
    const struct DDS_TypeAllocationParams_t * allocParams)
{
    UNUSED_ARG(allocParams);
    *self = NULL;
    return RTI_TRUE;
}

static RTIBool
RMW_Connext_TypeCodePtr_finalize_w_params(
    DDS_TypeCode **self,
    const struct DDS_TypeDeallocationParams_t * deallocParams)
{
    UNUSED_ARG(deallocParams);
    *self = NULL;
    return RTI_TRUE;
}

static RTIBool
RMW_Connext_TypeCodePtr_copy(
    DDS_TypeCode **dst,
    const DDS_TypeCode **src)
{
    *dst = const_cast<DDS_TypeCode*>(*src);
    return RTI_TRUE;
}

#define T                       DDS_TypeCode*
#define TSeq                    RMW_Connext_TypeCodePtrSeq
#define T_initialize_w_params   RMW_Connext_TypeCodePtr_initialize_w_params
#define T_finalize_w_params     RMW_Connext_TypeCodePtr_finalize_w_params
#define T_copy                  RMW_Connext_TypeCodePtr_copy
#include "dds_c/generic/dds_c_sequence_TSeq.gen"

static
DDS_TypeCode*
RMW_Connext_TypeCodePtrSeq_lookup_by_name(
    RMW_Connext_TypeCodePtrSeq *const self,
    const char *const name)
{
    const size_t seq_len = RMW_Connext_TypeCodePtrSeq_get_length(self);
    for (size_t i = 0; i < seq_len; i++)
    {
        DDS_TypeCode *const tc =
            *RMW_Connext_TypeCodePtrSeq_get_reference(self, i);
        if (nullptr == tc)
        {
            continue;
        }
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        const char *const tc_name = DDS_TypeCode_name(tc, &ex);
        if (nullptr == tc_name || DDS_NO_EXCEPTION_CODE != ex)
        {
            // ignore element
            continue;
        }
        if (strcmp(name, tc_name) == 0)
        {
            return tc;
        }
    }

    return nullptr;
}

static
RTIBool
RMW_Connext_TypeCodePtrSeq_assert_empty_slot(
    RMW_Connext_TypeCodePtrSeq *const self,
    size_t *const slot_i)
{
    const size_t seq_len = RMW_Connext_TypeCodePtrSeq_get_length(self);
    for (size_t i = 0; i < seq_len; i++)
    {
        DDS_TypeCode *const tc =
            *RMW_Connext_TypeCodePtrSeq_get_reference(self, i);
        if (nullptr == tc)
        {
            *slot_i = i;
            return RTI_TRUE;
        }
    }
    if (!RMW_Connext_TypeCodePtrSeq_ensure_length(
            self, seq_len + 1, seq_len + 1))
    {
        return RTI_FALSE;
    }
    *slot_i = seq_len;
    return RTI_TRUE;
}
static
RTIBool
RMW_Connext_TypeCodePtrSeq_append(
    RMW_Connext_TypeCodePtrSeq *const self,
    DDS_TypeCode *const tc)
{
    size_t slot_i = 0;

    if (!RMW_Connext_TypeCodePtrSeq_assert_empty_slot(self, &slot_i))
    {
        return RTI_FALSE;
    }

    *RMW_Connext_TypeCodePtrSeq_get_reference(self, slot_i) = tc;

    return RTI_TRUE;
}

static
DDS_TypeCode*
RMW_Connext_TypeCodePtrSeq_assert_from_ros(
    struct RMW_Connext_TypeCodePtrSeq *const self,
    const rosidl_message_type_support_t *const type_supports,
    const char *const type_name,
    DDS_TypeCode *const dds_tc = nullptr,
    bool *const added = nullptr)
{
    if (nullptr != added)
    {
        *added = false;
    }

    DDS_TypeCode *tc = nullptr;

    if (nullptr != type_name)
    {
        tc = RMW_Connext_TypeCodePtrSeq_lookup_by_name(self, type_name);

        if (nullptr != tc)
        {
            return tc;
        }
    }

    if (nullptr != dds_tc) {
        tc = dds_tc;
    } else {
        tc = rmw_connextdds_create_typecode(
                type_supports,
                type_name,
                nullptr /* intro_members */,
                false /* intro_members_cpp */,
                self);
        if (nullptr == tc)
        {
            return nullptr;
        }
    }

    if (!RMW_Connext_TypeCodePtrSeq_append(self, tc))
    {
        rmw_connextdds_delete_typecode(tc);
        return nullptr;
    }

    if (nullptr != added)
    {
        *added = true;
    }

    return tc;
}

static
void
RMW_Connext_TypeCodePtrSeq_finalize_elements(
    struct RMW_Connext_TypeCodePtrSeq *const self)
{
    const size_t seq_len = RMW_Connext_TypeCodePtrSeq_get_length(self);
    for (size_t i = 0; i < seq_len; i++)
    {
        DDS_TypeCode **const tc =
            RMW_Connext_TypeCodePtrSeq_get_reference(self, i);
        if (nullptr == *tc)
        {
            continue;
        }
        rmw_connextdds_delete_typecode(*tc);
        *tc = nullptr;
    }
}

#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
static
DDS_TCKind
rmw_connextdds_type_id_ros_to_dds(const uint8_t ros_type_id)
{
    switch (ros_type_id)
    {
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
    {
        return DDS_TK_BOOLEAN;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
    {
        return DDS_TK_OCTET;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    {
        return DDS_TK_CHAR;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
    {
        return DDS_TK_FLOAT;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
    {
        return DDS_TK_DOUBLE;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
    {
        return DDS_TK_SHORT;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
    {
        return DDS_TK_USHORT;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
    {
        return DDS_TK_LONG;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    {
        return DDS_TK_ULONG;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
    {
        return DDS_TK_LONGLONG;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    {
        return DDS_TK_ULONGLONG;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
    {
        return DDS_TK_STRING;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
    {
        return DDS_TK_WSTRING;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
    {
        return DDS_TK_STRUCT;
    }
    default:
    {
        RMW_CONNEXT_LOG_ERROR_A("unknown ROS type id: %d", ros_type_id)
        return DDS_TK_NULL;
    }
    }
}
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */

#define length_unbound          RTIXCdrLong_MAX

template<typename MemberType>
DDS_TypeCode*
rmw_connextdds_convert_type_member(
    DDS_TypeCodeFactory *const tc_factory,
    const MemberType *const member,
    struct RMW_Connext_TypeCodePtrSeq *const tc_cache)
{
    DDS_TypeCode *el_tc = nullptr;

    switch (member->type_id_)
    {
#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    {
        DDS_TCKind dds_type_id =
            rmw_connextdds_type_id_ros_to_dds(member->type_id_);

        el_tc =
            /* TODO(asorbini): refactor out this cast */
            const_cast<DDS_TypeCode*>(
                DDS_TypeCodeFactory_get_primitive_tc(tc_factory, dds_type_id));
        break;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
    {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_TypeCode *const el_tc_new =
                DDS_TypeCodeFactory_create_string_tc(
                    tc_factory,
                    (member->string_upper_bound_ > 0)?
                        member->string_upper_bound_ : length_unbound,
                    &ex);

        el_tc = RMW_Connext_TypeCodePtrSeq_assert_from_ros(
                    tc_cache,
                    nullptr /* members */,
                    nullptr /* name */,
                    el_tc_new);
        if (nullptr == el_tc)
        {
            return nullptr;
        }
        break;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
    {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_TypeCode *const el_tc_new =
                DDS_TypeCodeFactory_create_wstring_tc(
                    tc_factory,
                    (member->string_upper_bound_ > 0)?
                        member->string_upper_bound_ : length_unbound,
                    &ex);

        el_tc = RMW_Connext_TypeCodePtrSeq_assert_from_ros(
                    tc_cache,
                    nullptr /* members */,
                    nullptr /* name */,
                    el_tc_new);
        if (nullptr == el_tc)
        {
            return nullptr;
        }
        break;
    }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
    {
        bool cpp_version = false;
        const rosidl_message_type_support_t *const type_support_intro =
            RMW_Connext_MessageTypeSupport::get_type_support_intro(
                member->members_, cpp_version);
        if (nullptr == type_support_intro)
        {
            RMW_CONNEXT_LOG_ERROR_A(
                "introspection type support not found for member: %s",
                member->name_)
            return nullptr;
        }

        std::string type_name;

        if (cpp_version) {
            type_name = rmw_connextdds_create_type_name(
                reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
                    type_support_intro->data), true /* mangle_names */);
        } else {
            type_name = rmw_connextdds_create_type_name(
                reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(
                    type_support_intro->data), true /* mangle_names */);
        }

        el_tc = RMW_Connext_TypeCodePtrSeq_assert_from_ros(
                    tc_cache, member->members_, type_name.c_str());
        break;
    }
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */
    default:
    {
        RMW_CONNEXT_LOG_ERROR_A("unknown ROS type id: %d", member->type_id_)
        el_tc = nullptr;
    }
    }

    if (nullptr == el_tc)
    {
        RMW_CONNEXT_LOG_ERROR_A("failed to convert member to type code: %s",
            member->name_)
        return nullptr;
    }

    DDS_TypeCode *tc = el_tc;

    if (member->is_array_) {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        if (member->array_size_ > 0)
        {
            struct DDS_UnsignedLongSeq dimensions = DDS_SEQUENCE_INITIALIZER;
            if (!DDS_UnsignedLongSeq_ensure_length(&dimensions, 1, 1))
            {
                return nullptr;
            }
            *DDS_UnsignedLongSeq_get_reference(&dimensions, 0) = member->array_size_;
            DDS_TypeCode *const tc_array =
                DDS_TypeCodeFactory_create_array_tc(
                    tc_factory, &dimensions, el_tc, &ex);
            DDS_UnsignedLongSeq_finalize(&dimensions);

            tc = RMW_Connext_TypeCodePtrSeq_assert_from_ros(
                        tc_cache,
                        nullptr /* members */,
                        nullptr /* name */,
                        tc_array);
            if (nullptr == tc)
            {
                return nullptr;
            }
        } else {
            DDS_TypeCode *const tc_seq =
                DDS_TypeCodeFactory_create_sequence_tc(
                    tc_factory, length_unbound, el_tc, &ex);

            tc = RMW_Connext_TypeCodePtrSeq_assert_from_ros(
                        tc_cache,
                        nullptr /* members */,
                        nullptr /* name */,
                        tc_seq);
            if (nullptr == tc)
            {
                return nullptr;
            }
        }
    }

    return tc;
}

template<typename MembersType>
rmw_ret_t
rmw_connextdds_convert_type_members(
    DDS_TypeCodeFactory *const tc_factory,
    const MembersType *const members,
    struct DDS_StructMemberSeq *const tc_members,
    struct RMW_Connext_TypeCodePtrSeq *const tc_cache)
{
    if (members->member_count_ == 0)
    {
        /* empty type, add a single int32 member */

        return RMW_RET_OK;
    }
    if (!DDS_StructMemberSeq_ensure_length(
            tc_members, members->member_count_, members->member_count_))
    {
        return RMW_RET_ERROR;
    }
    for (uint32_t i = 0; i < members->member_count_; ++i)
    {
        DDS_StructMember *const tc_member =
            DDS_StructMemberSeq_get_reference(tc_members, i);
        const auto * member = members->members_ + i;

        /* Check that member has a non-empty name */
        size_t member_name_len = strlen(member->name_);
        if (nullptr == member->name_ || member_name_len == 0 ||
                (member_name_len == 1 &&
                    member->name_[member_name_len - 1] == '_'))
        {
            return RMW_RET_ERROR;
        }

        /* Names in the introspection plugin don't actually end with "_" */
        tc_member->name = DDS_String_dup(member->name_);
        if (nullptr == tc_member->name)
        {
            return RMW_RET_BAD_ALLOC;
        }

        tc_member->type =
            rmw_connextdds_convert_type_member(tc_factory, member, tc_cache);
        if (nullptr == tc_member->type)
        {
            return RMW_RET_ERROR;
        }
    }


    return RMW_RET_OK;
}


DDS_TypeCode*
rmw_connextdds_create_typecode(
    const rosidl_message_type_support_t *const type_supports,
    const char *const type_name,
    const void *const intro_members_in,
    const bool intro_members_cpp,
    RMW_Connext_TypeCodePtrSeq *const tc_cache)
{
    bool cpp_version = intro_members_cpp;
    const rosidl_message_type_support_t *intro_ts = nullptr;
    const void *intro_members = intro_members_in;
    struct RMW_Connext_TypeCodePtrSeq tc_cache_new = DDS_SEQUENCE_INITIALIZER;
    struct RMW_Connext_TypeCodePtrSeq *const tc_cache_new_ptr = &tc_cache_new,
                                      *tc_cache_ptr = tc_cache;
    if (nullptr == tc_cache_ptr)
    {
        tc_cache_ptr = tc_cache_new_ptr;
    }
    auto scope_exit_tc_cache_delete =
        rcpputils::make_scope_exit(
            [tc_cache_new_ptr]()
            {
                RMW_Connext_TypeCodePtrSeq_finalize_elements(tc_cache_new_ptr);
                RMW_Connext_TypeCodePtrSeq_finalize(tc_cache_new_ptr);
            });

    if (nullptr == intro_members_in)
    {
#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
        intro_ts =
            RMW_Connext_MessageTypeSupport::get_type_support_intro(
                type_supports, cpp_version);

        if (nullptr == intro_ts)
        {
            RMW_CONNEXT_LOG_ERROR_A("introspection type support not found for %s",
                type_name)
            return nullptr;
        }
        intro_members = intro_ts->data;
#else
        // Introspection type support must be available to generate a tc.
        UNUSED_ARG(intro_ts);
        UNUSED_ARG(type_supports);
        return nullptr;
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */
    }

    DDS_TypeCodeFactory *const tc_factory = DDS_TypeCodeFactory_get_instance();
    if (nullptr == tc_factory)
    {
        RMW_CONNEXT_LOG_ERROR("failed to get DDS_TypeCodeFactory")
        return nullptr;
    }

    struct DDS_StructMemberSeq tc_members = DDS_SEQUENCE_INITIALIZER;
    struct DDS_StructMemberSeq *const tc_members_ptr = &tc_members;
    auto scope_exit_tc_members_delete =
        rcpputils::make_scope_exit(
            [tc_members_ptr]()
            {
                const size_t seq_len =
                    DDS_StructMemberSeq_get_length(tc_members_ptr);
                for (size_t i = 0; i < seq_len; i++)
                {
                    DDS_StructMember *const tc_member =
                        DDS_StructMemberSeq_get_reference(tc_members_ptr, i);
                    DDS_String_free(tc_member->name);
                    tc_member->name = nullptr;
                }

                DDS_StructMemberSeq_finalize(tc_members_ptr);
            });

#if RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
    if (cpp_version) {
        const rosidl_typesupport_introspection_cpp::MessageMembers *const members =
            reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
                intro_members);

        if (RMW_RET_OK !=
                rmw_connextdds_convert_type_members(
                    tc_factory, members, &tc_members, tc_cache_ptr))
        {
            RMW_CONNEXT_LOG_ERROR_A("failed to convert members for %s",
                type_name)
            return nullptr;
        }
    } else {
        const rosidl_typesupport_introspection_c__MessageMembers *const members =
            reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(
                intro_members);

        if (RMW_RET_OK !=
                rmw_connextdds_convert_type_members(
                    tc_factory, members, &tc_members, tc_cache_ptr))
        {
            RMW_CONNEXT_LOG_ERROR_A("failed to convert members for %s",
                type_name)
            return nullptr;
        }
    }
#else
    UNUSED_ARG(cpp_version);
    UNUSED_ARG(intro_members);
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */


    DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
    DDS_TypeCode *tc =
        DDS_TypeCodeFactory_create_struct_tc(
            tc_factory, type_name, &tc_members, &ex);
    if (nullptr == tc)
    {
        RMW_CONNEXT_LOG_ERROR_A("failed to create type code for %s",
            type_name)
        return nullptr;
    }

    return tc;
}

void
rmw_connextdds_delete_typecode(DDS_TypeCode *const tc)
{
    DDS_TypeCodeFactory *const tc_factory = DDS_TypeCodeFactory_get_instance();
    if (nullptr == tc_factory)
    {
        RMW_CONNEXT_LOG_ERROR("failed to get DDS_TypeCodeFactory")
        return;
    }
    DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
    DDS_TypeCodeFactory_delete_tc(tc_factory, tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex)
    {
        RMW_CONNEXT_LOG_WARNING("failed to delete type code")
    }
}

void
rmw_connextdds_release_typecode_cache(
    RMW_Connext_TypeCodePtrSeq *const tc_cache)
{
    RMW_Connext_TypeCodePtrSeq_finalize_elements(tc_cache);
    RMW_Connext_TypeCodePtrSeq_finalize(tc_cache);
}
