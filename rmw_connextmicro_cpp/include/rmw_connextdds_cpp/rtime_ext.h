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

#ifndef RMW_CONNEXT__RTIME_EXT_H_
#define RMW_CONNEXT__RTIME_EXT_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

DDS_ReturnCode_t
DDS_DomainParticipant_lookup_type_pluginI(
    DDS_DomainParticipant *const self,
    const char *const type_name,
    struct DDS_TypePluginI **const plugin_out);

DDS_ReturnCode_t
DDS_DomainParticipant_is_type_in_use(
    DDS_DomainParticipant *const self,
    const char *const type_name,
    DDS_Boolean *const in_use_out);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RMW_CONNEXT__RTIME_EXT_H_ */