/* (c) 2020 Copyright, Real-Time Innovations, Inc. (RTI)
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
 */

#ifndef RTIROS__RTIME_EXT_H_
#define RTIROS__RTIME_EXT_H_

#include "rtiros/visibility_control.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

RTIROS_PUBLIC
DDS_ReturnCode_t
DDS_DomainParticipant_lookup_type_pluginI(
    DDS_DomainParticipant *const self,
    const char *const type_name,
    struct DDS_TypePluginI **const plugin_out);

RTIROS_PUBLIC
DDS_ReturnCode_t
DDS_DomainParticipant_is_type_in_use(
    DDS_DomainParticipant *const self,
    const char *const type_name,
    DDS_Boolean *const in_use_out);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RTIROS__RTIME_EXT_H_ */