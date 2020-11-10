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

#ifndef RMW_CONNEXTDDS__DDS_API_RTIME_HPP_
#define RMW_CONNEXTDDS__DDS_API_RTIME_HPP_

#include "rti_me_c.h"  // NOLINT(build/include)
#include "disc_dpde/disc_dpde_discovery_plugin.h"
#include "wh_sm/wh_sm_history.h"
#include "rh_sm/rh_sm_history.h"
#include "netio/netio_udp.h"
#include "netio_shmem/netio_shmem.h"
// #include "sec_core/sec_core_c.h"
#include "rmw_connextdds/rtime_ext.h"
#include "REDASequence.h"

typedef DDS_UntypedSampleSeq RMW_Connext_UntypedSampleSeq;

#define RMW_Connext_UntypedSampleSeq_INITIALIZER    DDS_SEQUENCE_INITIALIZER

#endif  // RMW_CONNEXTDDS__DDS_API_RTIME_HPP_
