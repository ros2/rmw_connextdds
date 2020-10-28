/******************************************************************************
 *
 * (c) Copyright, Real-Time Innovations, 2019-2020.
 *
 * All rights reserved.
 * No duplications, whole or partial, manual or electronic, may be made without
 * express written permission.  Any such copies, or revisions thereof, must 
 * display this notice unaltered.
 * This code contains trade secrets of Real-Time Innovations, Inc.
 *
 ******************************************************************************/

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