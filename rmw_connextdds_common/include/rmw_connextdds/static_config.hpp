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

#ifndef RMW_CONNEXTDDS__STATIC_CONFIG_HPP_
#define RMW_CONNEXTDDS__STATIC_CONFIG_HPP_

/******************************************************************************
 * Default User Configuration
 ******************************************************************************/
#ifndef RMW_CONNEXT_DEFAULT_DOMAIN
#define RMW_CONNEXT_DEFAULT_DOMAIN                 0u
#endif /* RMW_CONNEXT_DEFAULT_DOMAIN */

#ifndef RMW_CONNEXT_DEFAULT_UDP_INTERFACE
#define RMW_CONNEXT_DEFAULT_UDP_INTERFACE          "lo"
#endif /* RMW_CONNEXT_DEFAULT_UDP_INTERFACE */

#ifndef RMW_CONNEXT_DEFAULT_QOS_LIBRARY
#define RMW_CONNEXT_DEFAULT_QOS_LIBRARY            "ros2"
#endif /* RMW_CONNEXT_DEFAULT_QOS_LIBRARY */

/******************************************************************************
 * Environment Variables
 ******************************************************************************/
#ifndef RMW_CONNEXT_ENV_UDP_INTERFACE
#define RMW_CONNEXT_ENV_UDP_INTERFACE \
  "RMW_CONNEXT_UDP_INTERFACE"
#endif /* RMW_CONNEXT_ENV_UDP_INTERFACE */

#ifndef RMW_CONNEXT_ENV_INITIAL_PEERS
#define RMW_CONNEXT_ENV_INITIAL_PEERS \
  "RMW_CONNEXT_INITIAL_PEERS"
#endif /* RMW_CONNEXT_ENV_INITIAL_PEERS */

#ifndef RMW_CONNEXT_ENV_QOS_LIBRARY
#define RMW_CONNEXT_ENV_QOS_LIBRARY \
  "RMW_CONNEXT_QOS_LIBRARY"
#endif /* RMW_CONNEXT_ENV_QOS_LIBRARY */

#ifndef RMW_CONNEXT_ENV_USE_DEFAULT_PUBLISH_MODE
#define RMW_CONNEXT_ENV_USE_DEFAULT_PUBLISH_MODE \
  "RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE"
#endif /* RMW_CONNEXT_ENV_USE_DEFAULT_PUBLISH_MODE */

#ifndef RMW_CONNEXT_ENV_REQUEST_REPLY_MAPPING
#define RMW_CONNEXT_ENV_REQUEST_REPLY_MAPPING \
  "RMW_CONNEXT_REQUEST_REPLY_MAPPING"
#endif /* RMW_CONNEXT_ENV_REQUEST_REPLY_MAPPING */

#ifndef RMW_CONNEXT_ENV_CYCLONE_COMPATIBILITY_MODE
#define RMW_CONNEXT_ENV_CYCLONE_COMPATIBILITY_MODE \
  "RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE"
#endif /* RMW_CONNEXT_ENV_REQUEST_REPLY_MAPPING */

#ifndef RMW_CONNEXT_ENV_OLD_RMW_COMPATIBILITY_MODE
#define RMW_CONNEXT_ENV_OLD_RMW_COMPATIBILITY_MODE \
  "RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE"
#endif /* RMW_CONNEXT_ENV_OLD_RMW_COMPATIBILITY_MODE */

#ifndef RMW_CONNEXT_ENV_DISABLE_FAST_ENDPOINT_DISCOVERY
#define RMW_CONNEXT_ENV_DISABLE_FAST_ENDPOINT_DISCOVERY \
  "RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY"
#endif /* RMW_CONNEXT_ENV_OLD_RMW_COMPATIBILITY_MODE */

#ifndef RMW_CONNEXT_ENV_DISABLE_LARGE_DATA_OPTIMIZATIONS
#define RMW_CONNEXT_ENV_DISABLE_LARGE_DATA_OPTIMIZATIONS \
  "RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS"
#endif /* RMW_CONNEXT_ENV_DISABLE_LARGE_DATA_OPTIMIZATIONS */

// TODO(security-wg): These are intended to be temporary, and need to be
// refactored into a proper abstraction.
#ifndef RMW_CONNEXT_ENV_SECURITY_LOG_FILE
#define RMW_CONNEXT_ENV_SECURITY_LOG_FILE     "ROS_SECURITY_LOG_FILE"
#endif /* RMW_CONNEXT_ENV_SECURITY_LOG_FILE */

#ifndef RMW_CONNEXT_ENV_SECURITY_LOG_PUBLISH
#define RMW_CONNEXT_ENV_SECURITY_LOG_PUBLISH     "ROS_SECURITY_LOG_PUBLISH"
#endif /* RMW_CONNEXT_ENV_SECURITY_LOG_PUBLISH */

#ifndef RMW_CONNEXT_ENV_SECURITY_LOG_VERBOSITY
#define RMW_CONNEXT_ENV_SECURITY_LOG_VERBOSITY     "ROS_SECURITY_LOG_VERBOSITY"
#endif /* RMW_CONNEXT_ENV_SECURITY_LOG_VERBOSITY */

#ifndef RMW_CONNEXT_ENV_ENDPOINT_QOS_OVERRIDE_POLICY
#define RMW_CONNEXT_ENV_ENDPOINT_QOS_OVERRIDE_POLICY     "RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY"
#endif /* RMW_CONNEXT_ENV_ALLOW_TOPIC_QOS_PROFILES */


/******************************************************************************
 * DDS Implementation
 * Select the DDS implementation used to build the RMW library.
 ******************************************************************************/
#define RMW_CONNEXT_DDS_API_PRO         0
#define RMW_CONNEXT_DDS_API_MICRO       1

#ifndef RMW_CONNEXT_DDS_API
#define RMW_CONNEXT_DDS_API             RMW_CONNEXT_DDS_API_PRO
#endif /* RMW_CONNEXT_DDS_API */

#ifndef RMW_CONNEXT_DDS_API_PRO_LEGACY
#define RMW_CONNEXT_DDS_API_PRO_LEGACY  0
#endif /* RMW_CONNEXT_DDS_API_PRO_LEGACY */

#ifndef RMW_CONNEXT_ENABLE_SECURITY
#define RMW_CONNEXT_ENABLE_SECURITY \
  (RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_PRO)
#endif /* RMW_CONNEXT_ENABLE_SECURITY */

/******************************************************************************
 * Log configuration.
 * This option controls the logging output of the RMW and which logging
 * calls will actually be compiled into the library:
 *   - NONE: disable all logging output
 *   - DEFAULT: include only INFO to ERROR log calls, log through rcutils
 *   - ALL: include all log calls, log through rcutils
 *   - PRINTF: include all log calls, log through printf()
 * Keep in mind that compiling all logging statements, including those at
 * verbosity debug and higher,will likely cause some performance degradation.
 ******************************************************************************/
#define RMW_CONNEXT_LOG_MODE_NONE       0
#define RMW_CONNEXT_LOG_MODE_DEFAULT    1
#define RMW_CONNEXT_LOG_MODE_ALL        2
#define RMW_CONNEXT_LOG_MODE_PRINTF     3

#ifndef RMW_CONNEXT_LOG_MODE
#define RMW_CONNEXT_LOG_MODE            RMW_CONNEXT_LOG_MODE_DEFAULT
#endif /* RMW_CONNEXT_LOG_MODE */

/******************************************************************************
 * Code assertion.
 * Enable various precondition assertions in the code.
 ******************************************************************************/
#ifndef RMW_CONNEXT_ASSERT_ENABLE
#define RMW_CONNEXT_ASSERT_ENABLE       0
#endif /* RMW_CONNEXT_ASSERT_ENABLE */

/******************************************************************************
 * If this option is enabled, the RMW will always implement RPC requests between
 * Clients and Servers using the "basic" mapping from DDS-RPC, which serializes
 * a "request header" (containing GUID and SN) before the message payload,
 * This allows the RMW to interoperate with DDS implementations which don't
 * support the "extended" mapping via propagation and correlation of
 * "sample identities".
 * This option is enabled by default only for RTI Connext DDS Micro.
 ******************************************************************************/
#ifndef RMW_CONNEXT_FORCE_REQUEST_REPLY_MAPPING_BASIC
#define RMW_CONNEXT_FORCE_REQUEST_REPLY_MAPPING_BASIC \
  (RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO)
#endif /* RMW_CONNEXT_FORCE_REQUEST_REPLY_MAPPING_BASIC */

/******************************************************************************
 * Shmem Transport.
 * If disabled, the shared memory transport will not be used by the
 * DomainParticipant (Micro version only).
 ******************************************************************************/
#ifndef RMW_CONNEXT_TRANSPORT_SHMEM
#define RMW_CONNEXT_TRANSPORT_SHMEM     1
#endif /* RMW_CONNEXT_TRANSPORT_SHMEM */

/******************************************************************************
 * Always use UUID algorithm to generate the RTPS GUID of the DomainParticipant.
 ******************************************************************************/
#ifndef RMW_CONNEXT_RTPS_AUTO_ID_FROM_UUID
#define RMW_CONNEXT_RTPS_AUTO_ID_FROM_UUID   1
#endif /* RMW_CONNEXT_RTPS_AUTO_ID_FROM_UUID */

/******************************************************************************
 * Address used to constrain the UDPv4 transport to only allow "localhost"
 * communication. It must match the address of the localhost interface.
 ******************************************************************************/
#ifndef RMW_CONNEXT_LOCALHOST_ONLY_ADDRESS
#define RMW_CONNEXT_LOCALHOST_ONLY_ADDRESS   "127.0.0.1"
#endif /* RMW_CONNEXT_LOCALHOST_ONLY_ADDRESS */

/******************************************************************************
 * Maximum length of a ContentFilterProperty_t.
 * According to the RTPS spec, ContentFilterProperty_t has the following fields:
 * -contentFilteredTopicName (max length 256)
 * -relatedTopicName (max length 256)
 * -filterClassName (max length 256)
 * -filterName (DDSSQL)
 * -filterExpression
 * In Connext, contentfilter_property_max_length is sum of lengths of all these
 * fields, which by default is 256.
 * So we set the limit to 1024, to accomodate the complete topic name with
 * namespaces.
 * The property will only be set if the current max length is smaller.
 ******************************************************************************/
#ifndef RMW_CONNEXT_CONTENTFILTER_PROPERTY_MAX_LENGTH
#define RMW_CONNEXT_CONTENTFILTER_PROPERTY_MAX_LENGTH   1024
#endif /* RMW_CONNEXT_CONTENTFILTER_PROPERTY_MAX_LENGTH */

/******************************************************************************
 * Maximum serialized size of a type code propagated by the RMW.
 ******************************************************************************/
#ifndef RMW_CONNEXT_TYPE_CODE_MAX_SERIALIZED_SIZE
#define RMW_CONNEXT_TYPE_CODE_MAX_SERIALIZED_SIZE   0
#endif /* RMW_CONNEXT_TYPE_CODE_MAX_SERIALIZED_SIZE */

/******************************************************************************
 * Maximum serialized size of a type object propagated by the RMW.
 ******************************************************************************/
#ifndef RMW_CONNEXT_TYPE_OBJECT_MAX_SERIALIZED_SIZE
#define RMW_CONNEXT_TYPE_OBJECT_MAX_SERIALIZED_SIZE   65000
#endif /* RMW_CONNEXT_TYPE_OBJECT_MAX_SERIALIZED_SIZE */

/******************************************************************************
 * Automatically tune DataWriterQos to better handle reliable "large data".
 ******************************************************************************/
#ifndef RMW_CONNEXT_DEFAULT_LARGE_DATA_OPTIMIZATIONS
#define RMW_CONNEXT_DEFAULT_LARGE_DATA_OPTIMIZATIONS     1
#endif /* RMW_CONNEXT_DEFAULT_LARGE_DATA_OPTIMIZATIONS */

/******************************************************************************
 * Threshold of the maximum serialized size for a message to be considered
 * "large". The RMW will automatically tune the reliability protocol for
 * writers of types that match or exceed this threshold.
 ******************************************************************************/
#ifndef RMW_CONNEXT_LARGE_DATA_MIN_SERIALIZED_SIZE
#define RMW_CONNEXT_LARGE_DATA_MIN_SERIALIZED_SIZE   1048576 /* 1MB */
#endif /* RMW_CONNEXT_LARGE_DATA_MIN_SERIALIZED_SIZE */

/******************************************************************************
 * Size of the "send window" of an RTPS Writer sending "large data".
 ******************************************************************************/
#ifndef RMW_CONNEXT_LARGE_DATA_SEND_WINDOW_SIZE_MIN
#define RMW_CONNEXT_LARGE_DATA_SEND_WINDOW_SIZE_MIN   10
#endif /* RMW_CONNEXT_LARGE_DATA_SEND_WINDOW_SIZE_MIN */

#ifndef RMW_CONNEXT_LARGE_DATA_SEND_WINDOW_SIZE_MAX
#define RMW_CONNEXT_LARGE_DATA_SEND_WINDOW_SIZE_MAX   100
#endif /* RMW_CONNEXT_LARGE_DATA_SEND_WINDOW_SIZE_MAX */

/******************************************************************************
 * Regular hearbeat period used by an RTPS Writer sending "large data".
 * This is an initializer for an instance of type DDS_Duration_t.
 ******************************************************************************/
#ifndef RMW_CONNEXT_LARGE_DATA_HEARTBEAT_PERIOD
#define RMW_CONNEXT_LARGE_DATA_HEARTBEAT_PERIOD       {0, 200000000}   /* 200ms */
#endif /* RMW_CONNEXT_LARGE_DATA_HEARTBEAT_PERIOD */

/******************************************************************************
 * Fast hearbeat period used by an RTPS Writer sending "large data" to allow
 * late joiners and out of sync readers to catch.
 * This is an initializer for an instance of type DDS_Duration_t.
 ******************************************************************************/
#ifndef RMW_CONNEXT_LARGE_DATA_HEARTBEAT_PERIOD_FAST
#define RMW_CONNEXT_LARGE_DATA_HEARTBEAT_PERIOD_FAST  {0, 20000000}   /* 20ms */
#endif /* RMW_CONNEXT_LARGE_DATA_HEARTBEAT_PERIOD_FAST */

/******************************************************************************
 * In order to reduce the time to cleanup a participant (Node), we use the
 * advice from
 * https://community.rti.com/static/documentation/connext-dds/5.3.1/doc/api/connext_dds/api_cpp/structDDS__DomainParticipantQos.html
 * and reduce the shutdown_cleanup_period to 50 milliseconds.
 ******************************************************************************/
#ifndef RMW_CONNEXT_SHUTDOWN_CLEANUP_PERIOD_SEC
#define RMW_CONNEXT_SHUTDOWN_CLEANUP_PERIOD_SEC   0
#endif /* RMW_CONNEXT_SHUTDOWN_CLEANUP_PERIOD_SEC */

#ifndef RMW_CONNEXT_SHUTDOWN_CLEANUP_PERIOD_NSEC
#define RMW_CONNEXT_SHUTDOWN_CLEANUP_PERIOD_NSEC   50000000
#endif /* RMW_CONNEXT_SHUTDOWN_CLEANUP_PERIOD_NSEC */

/******************************************************************************
 * Modify DomainParticipantQos to perform faster endpoint discovery
 ******************************************************************************/
#ifndef RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY
#define RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY     1
#endif /* RMW_CONNEXT_FAST_ENDPOINT_DISCOVERY */

/******************************************************************************
 * Override dds.transport.UDPv4.builtin.ignore_loopback_interface in
 * DomainParticipantQos to force communication over loopback (in addition to
 * other transports, e.g. shared memory).
 ******************************************************************************/
#ifndef RMW_CONNEXT_DONT_IGNORE_LOOPBACK_INTERFACE
#define RMW_CONNEXT_DONT_IGNORE_LOOPBACK_INTERFACE     0
#endif /* RMW_CONNEXT_DONT_IGNORE_LOOPBACK_INTERFACE */

/******************************************************************************
 * Enable support for running in "compatibility mode" with the previous RMW for
 * Connext (rmw_connext_cpp). If this option is enable, the mode can be
 * enabled via env variable. Once the mode is enabled, rmw_connextdds will:
 * - Add suffix "_" to member names of types propagated via DDS discovery.
 ******************************************************************************/
#ifndef RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE
#define RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE     1
#endif /* RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE */

/******************************************************************************
 * Use an alternative implementation of WaitSets based on C++ std library
 ******************************************************************************/
#ifndef RMW_CONNEXT_CPP_STD_WAITSETS
#define RMW_CONNEXT_CPP_STD_WAITSETS     0
#endif /* RMW_CONNEXT_CPP_STD_WAITSETS */

/******************************************************************************
 * ROS Target Release
 ******************************************************************************/
#define RMW_CONNEXT_RELEASE_DASHING     10
#define RMW_CONNEXT_RELEASE_ELOQUENT    20
#define RMW_CONNEXT_RELEASE_FOXY        30
#define RMW_CONNEXT_RELEASE_ROLLING     40

#ifndef RMW_CONNEXT_RELEASE
#define RMW_CONNEXT_RELEASE             RMW_CONNEXT_RELEASE_ROLLING
#endif /* RMW_CONNEXT_RELEASE */

#ifndef RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#define RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON \
  (RMW_CONNEXT_RELEASE >= RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

#ifndef RMW_CONNEXT_HAVE_SCOPE_EXIT
#define RMW_CONNEXT_HAVE_SCOPE_EXIT \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_SCOPE_EXIT */

#ifndef RMW_CONNEXT_HAVE_GET_DOMAIN
#define RMW_CONNEXT_HAVE_GET_DOMAIN \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_SCOPE_EXIT */

#ifndef RMW_CONNEXT_HAVE_OPTIONS
#define RMW_CONNEXT_HAVE_OPTIONS \
  (RMW_CONNEXT_RELEASE >= RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_OPTIONS */

#ifndef RMW_CONNEXT_HAVE_OPTIONS_PUBSUB
#define RMW_CONNEXT_HAVE_OPTIONS_PUBSUB \
  (RMW_CONNEXT_RELEASE >= RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_OPTIONS_PUBSUB */

#ifndef RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS
#define RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS */

#ifndef RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT
#define RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT */

#ifndef RMW_CONNEXT_HAVE_LIFESPAN_QOS
#define RMW_CONNEXT_HAVE_LIFESPAN_QOS \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_LIFESPAN_QOS */

#ifndef RMW_CONNEXT_HAVE_MESSAGE_LOST
#define RMW_CONNEXT_HAVE_MESSAGE_LOST \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_MESSAGE_LOST */

#ifndef RMW_CONNEXT_HAVE_SERVICE_INFO
#define RMW_CONNEXT_HAVE_SERVICE_INFO \
  (RMW_CONNEXT_RELEASE >= RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */

#ifndef RMW_CONNEXT_HAVE_LOCALHOST_ONLY
#define RMW_CONNEXT_HAVE_LOCALHOST_ONLY \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_LOCALHOST_ONLY */

#ifndef RMW_CONNEXT_HAVE_TAKE_SEQ
#define RMW_CONNEXT_HAVE_TAKE_SEQ \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_TAKE_SEQ */

#ifndef RMW_CONNEXT_HAVE_SECURITY
#define RMW_CONNEXT_HAVE_SECURITY \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_SECURITY */

#ifndef RMW_CONNEXT_HAVE_COMMON_MUTEX
#define RMW_CONNEXT_HAVE_COMMON_MUTEX       RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#endif /* RMW_CONNEXT_HAVE_COMMON_MUTEX */

#ifndef RMW_CONNEXT_HAVE_LOAN_MESSAGE
#define RMW_CONNEXT_HAVE_LOAN_MESSAGE \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_LOAN_MESSAGE */

#ifndef RMW_CONNEXT_HAVE_MESSAGE_INFO_TS
#define RMW_CONNEXT_HAVE_MESSAGE_INFO_TS \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_MESSAGE_INFO_TS */

#ifndef RMW_CONNEXT_HAVE_GET_INFO_BY_TOPIC
#define RMW_CONNEXT_HAVE_GET_INFO_BY_TOPIC \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_ELOQUENT)
#endif /* RMW_CONNEXT_HAVE_GET_INFO_BY_TOPIC */

#ifndef RMW_CONNEXT_HAVE_QOS_PROFILE_API
#define RMW_CONNEXT_HAVE_QOS_PROFILE_API \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_QOS_PROFILE_API */

#ifndef RMW_CONNEXT_HAVE_TIME_UTILS
#define RMW_CONNEXT_HAVE_TIME_UTILS \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_TIME_UTILS */

#include "resource_limits.hpp"

#endif  // RMW_CONNEXTDDS__STATIC_CONFIG_HPP_
