// Copyright 2021 Real-Time Innovations, Inc. (RTI)
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
#ifndef RTI_CONNEXTDDS_CUSTOM_SQL_FILTER__CUSTOM_SQL_FILTER_HPP_
#define RTI_CONNEXTDDS_CUSTOM_SQL_FILTER__CUSTOM_SQL_FILTER_HPP_

#include "rti_connext_dds_custom_sql_filter/visibility_control.h"
#include "ndds/ndds_c.h"

namespace rti_connext_dds_custom_sql_filter
{

RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC
DDS_ReturnCode_t
register_content_filter(DDS_DomainParticipant * const participant);

RTI_CONNEXTDDS_CUSTOM_SQL_FILTER_PUBLIC
extern const char * const PLUGIN_NAME;

}  // namespace rti_connext_dds_custom_sql_filter

#endif  // RTI_CONNEXTDDS_CUSTOM_SQL_FILTER__CUSTOM_SQL_FILTER_HPP_
