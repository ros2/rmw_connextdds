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

#ifndef RMW_CONNEXT__STATIC_CONFIG_HPP_
#define RMW_CONNEXT__STATIC_CONFIG_HPP_

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
 ******************************************************************************/
#define RMW_CONNEXT_DDS_API_MICRO       0
#define RMW_CONNEXT_DDS_API_PRO         1

#ifndef RMW_CONNEXT_DDS_API
#define RMW_CONNEXT_DDS_API             RMW_CONNEXT_DDS_API_MICRO
#endif /* RMW_CONNEXT_DDS_API */

/******************************************************************************
 * Log configuration
 ******************************************************************************/
#ifndef RMW_CONNEXT_LOG_MODE
#define RMW_CONNEXT_LOG_MODE            RMW_CONNEXT_LOG_MODE_DEFAULT
#endif /* RMW_CONNEXT_LOG_MODE */

#ifndef RMW_CONNEXT_ASSERT_ENABLE
#define RMW_CONNEXT_ASSERT_ENABLE       0
#endif /* RMW_CONNEXT_ASSERT_ENABLE */

/******************************************************************************
 * Request/Reply support
 ******************************************************************************/
#ifndef RMW_CONNEXT_EMULATE_REQUESTREPLY
#define RMW_CONNEXT_EMULATE_REQUESTREPLY \
        (RMW_CONNEXT_DDS_API == RMW_CONNEXT_DDS_API_MICRO)
#endif /* RMW_CONNEXT_EMULATE_REQUESTREPLY */

/******************************************************************************
 ******************************************************************************
 * Experimental features below
 ******************************************************************************
 ******************************************************************************/


/******************************************************************************
 * Qos Profile support
 ******************************************************************************/
#ifndef RMW_CONNEXT_USE_PROFILES
#define RMW_CONNEXT_USE_PROFILES        0
#endif /* RMW_CONNEXT_USE_PROFILES */

#ifndef RMW_CONNEXT_EXPORT_MESSAGE_TYPES
#define RMW_CONNEXT_EXPORT_MESSAGE_TYPES        1
#endif /* RMW_CONNEXT_EXPORT_MESSAGE_TYPES */

#ifndef RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES
#define RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES    0
#endif /* RMW_CONNEXT_COMPATIBLE_MESSAGE_TYPES */

#define RMW_CONNEXT_RELEASE_ROLLING     0
#define RMW_CONNEXT_RELEASE_FOXY        1

/******************************************************************************
 * ROS Target Release
 ******************************************************************************/
#ifndef RMW_CONNEXT_RELEASE
#define RMW_CONNEXT_RELEASE             RMW_CONNEXT_RELEASE_ROLLING
#endif /* RMW_CONNEXT_RELEASE */

#include "resource_limits.hpp"

#endif /* RMW_CONNEXT__STATIC_CONFIG_HPP_ */
