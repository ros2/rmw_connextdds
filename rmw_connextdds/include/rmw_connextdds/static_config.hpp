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

/******************************************************************************
 * Environment Variables
 ******************************************************************************/
#ifndef RMW_CONNEXT_ENV_UDP_INTERFACE
#define RMW_CONNEXT_ENV_UDP_INTERFACE   "RMW_CONNEXT_UDP_INTERFACE"
#endif /* RMW_CONNEXT_ENV_UDP_INTERFACE */

#ifndef RMW_CONNEXT_ENV_INITIAL_PEER
#define RMW_CONNEXT_ENV_INITIAL_PEER    "RMW_CONNEXT_INITIAL_PEER"
#endif /* RMW_CONNEXT_ENV_INITIAL_PEER */

#ifndef RMW_CONNEXT_ENV_DEFAULT_RELIABLE_READER_PROFILE
#define RMW_CONNEXT_ENV_DEFAULT_RELIABLE_READER_PROFILE \
  "RMW_CONNEXT_DEFAULT_RELIABLE_READER_PROFILE"
#endif /* RMW_CONNEXT_ENV_DEFAULT_RELIABLE_READER_PROFILE */

#ifndef RMW_CONNEXT_ENV_DEFAULT_BESTEFFORT_READER_PROFILE
#define RMW_CONNEXT_ENV_DEFAULT_BESTEFFORT_READER_PROFILE \
  "RMW_CONNEXT_DEFAULT_BESTEFFORT_READER_PROFILE"
#endif /* RMW_CONNEXT_DEFAULT_BESTEFFORT_READER_PROFILE */

/******************************************************************************
 * DDS Implementation
 * Select the DDS implementation used to build the RMW library.
 ******************************************************************************/
#define RMW_CONNEXT_DDS_API_PRO         0
#define RMW_CONNEXT_DDS_API_MICRO       1

#ifndef RMW_CONNEXT_DDS_API
#define RMW_CONNEXT_DDS_API             RMW_CONNEXT_DDS_API_PRO
#endif /* RMW_CONNEXT_DDS_API */

/******************************************************************************
 * Log configuration.
 * This option controls the logging output of the RMW and which logging
 * calls will actually be compiled into the library:
 *   - NONE: disable all logging
 *   - DEFAULT: include INFO to ERROR log calls, log through rclutils
 *   - ALL: include all log calls, log through rclutils
 *   - PRINTF: include all log calls, log through printf()
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
 * Request/Reply support.
 * If this option is enabled, the RMW will implement RPC requests between
 * Clients and Servers using a custom protocol that serializes a "request header"
 * (containing GUID and SN) before the message payload, instead of using the
 * standard DDS RPC protocol. This allows the RMW to interoperate with DDS
 * implementations which don't support propagation and correlation of
 * "sample identity".
 * This option is enabled by default only for RTI Connext DDS Micro, and it must
 * be enable at build time not interoperable with the Pro version.
 ******************************************************************************/
#ifndef RMW_CONNEXT_EMULATE_REQUESTREPLY
#define RMW_CONNEXT_EMULATE_REQUESTREPLY \
  (RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO)
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */

/******************************************************************************
 * Shmem Transport.
 * If disabled, the shared memory transport will not be used by the
 * DomainParticipant.
 ******************************************************************************/
#ifndef RMW_CONNEXT_TRANSPORT_SHMEM
#define RMW_CONNEXT_TRANSPORT_SHMEM     1
#endif /* RMW_CONNEXT_TRANSPORT_SHMEM */

/******************************************************************************
 * ROS Target Release
 ******************************************************************************/
#define RMW_CONNEXT_RELEASE_DASHING     10
#define RMW_CONNEXT_RELEASE_ELOQUENT    20
#define RMW_CONNEXT_RELEASE_FOXY        30
#define RMW_CONNEXT_RELEASE_ROLLING     40

#ifndef RMW_CONNEXT_RELEASE
#define RMW_CONNEXT_RELEASE             RMW_CONNEXT_RELEASE_DASHING
#endif /* RMW_CONNEXT_RELEASE */

#ifndef RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#define RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON \
  (RMW_CONNEXT_RELEASE >= RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON */

#ifndef RMW_CONNEXT_HAVE_SCOPE_EXIT
#define RMW_CONNEXT_HAVE_SCOPE_EXIT \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_SCOPE_EXIT */

#ifndef RMW_CONNEXT_HAVE_OPTIONS
#define RMW_CONNEXT_HAVE_OPTIONS \
  (RMW_CONNEXT_RELEASE >= RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_OPTIONS */

#ifndef RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS
#define RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY)
#endif /* RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS */

#ifndef RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT
#define RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_INCOMPATIBLE_QOS_EVENT */

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
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_LOCALHOST_ONLY */

#ifndef RMW_CONNEXT_HAVE_TAKE_SEQ
#define RMW_CONNEXT_HAVE_TAKE_SEQ \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_TAKE_SEQ */

#ifndef RMW_CONNEXT_HAVE_SECURITY
#define RMW_CONNEXT_HAVE_SECURITY \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_SECURITY */

#ifndef RMW_CONNEXT_HAVE_COMMON_MUTEX
#define RMW_CONNEXT_HAVE_COMMON_MUTEX       RMW_CONNEXT_HAVE_PKG_RMW_DDS_COMMON
#endif /* RMW_CONNEXT_HAVE_COMMON_MUTEX */

#ifndef RMW_CONNEXT_HAVE_LOAN_MESSAGE
#define RMW_CONNEXT_HAVE_LOAN_MESSAGE \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_LOAN_MESSAGE */

#ifndef RMW_CONNEXT_HAVE_MESSAGE_INFO_TS
#define RMW_CONNEXT_HAVE_MESSAGE_INFO_TS \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_MESSAGE_INFO_TS */

#ifndef RMW_CONNEXT_HAVE_GET_INFO_BY_TOPIC
#define RMW_CONNEXT_HAVE_GET_INFO_BY_TOPIC \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_GET_INFO_BY_TOPIC */

#ifndef RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT
#define RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT \
  (RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_DASHING)
#endif /* RMW_CONNEXT_HAVE_INTRO_TYPE_SUPPORT */


#include "resource_limits.hpp"

#endif  // RMW_CONNEXTDDS__STATIC_CONFIG_HPP_
