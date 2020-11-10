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

#ifndef RMW_CONNEXTDDS__DDS_API_NDDS_HPP_
#define RMW_CONNEXTDDS__DDS_API_NDDS_HPP_

#include "ndds/ndds_c.h"

#include "rcutils/types.h"

DDS_SEQUENCE(RMW_Connext_Uint8ArrayPtrSeq, rcutils_uint8_array_t *);

typedef RMW_Connext_Uint8ArrayPtrSeq RMW_Connext_UntypedSampleSeq;

#define RMW_Connext_UntypedSampleSeq_INITIALIZER    DDS_SEQUENCE_INITIALIZER

#define DDS_UntypedSampleSeq_get_reference(seq_, i_) \
  *RMW_Connext_Uint8ArrayPtrSeq_get_reference(seq_, i_)

#define DDS_UntypedSampleSeq_get_length(seq_) \
  RMW_Connext_Uint8ArrayPtrSeq_get_length(seq_)

#if RMW_CONNEXT_DDS_API_PRO_LEGACY
#define RTIXCdrLong_MAX 2147483647
#endif /* RMW_CONNEXT_DDS_API_PRO_LEGACY */

#endif  // RMW_CONNEXTDDS__DDS_API_NDDS_HPP_
