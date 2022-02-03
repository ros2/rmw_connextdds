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

#include <fstream>
#include <sstream>
#include <string>

#include "rmw_connextdds/rmw_impl.hpp"

rmw_ret_t
RMW_Connext_UserResourceLimits::load(const DDS_StringSeq *const file_names)
{
  DDS_Long files_len = 0;

  files_len = DDS_StringSeq_get_length(file_names);
  
  DDS_StringSeq line_tokens = DDS_SEQUENCE_INITIALIZER;
  auto scope_exit_line_tokens = rcpputils::make_scope_exit(
    [&line_tokens]()
    {
      if (!DDS_StringSeq_finalize(&line_tokens)) {
        RMW_CONNEXT_LOG_ERROR("failed to finalize line tokens")
      }
    });

  for (DDS_Long i = 0; i < files_len; i++) {
    const char * file_name = *DDS_StringSeq_get_reference(file_names, i);

    if (nullptr == file_name) {
      RMW_CONNEXT_LOG_ERROR_SET("invalid NULL file name")
      return RMW_RET_ERROR;
    }
    
    try {
      std::ifstream file_stream(file_name);
      std::string file_content(
        std::istreambuf_iterator<char>(file_stream),
        std::istreambuf_iterator<char>());

      std::string line;
      while (std::getline(file_stream, line))
      {
        if (!DDS_StringSeq_set_length(&line_tokens, 0)) {
          RMW_CONNEXT_LOG_ERROR_SET("failed to set length of tokens sequence")
          return RMW_RET_ERROR;
        }
        rmw_ret_t rc = rmw_connextdds_parse_string_list(
          line.c_str(),
          &line_tokens,
          ',' /* delimiter */,
          true /* trim_elements */,
          false /* allow_empty_elements */,
          false /* append_values */);
        if (RMW_RET_OK != rc) {
          RMW_CONNEXT_LOG_ERROR_A_SET(
            "failed to parse resource limits line: '%s'", line.c_str())
          return RMW_RET_ERROR;
        }
        DDS_Long line_len = DDS_StringSeq_get_length(&line_tokens);
        if (line_len <= 0)
        {
          continue;
        }

        std::string line_kind = *DDS_StringSeq_get_reference(&line_tokens, 0);

        if (line_kind == "serialized_size_max") {
          std::string type_name = *DDS_StringSeq_get_reference(&line_tokens, 1);
          std::string type_ssm_str = *DDS_StringSeq_get_reference(&line_tokens, 2);
          std::stringstream type_ssm_stream(type_ssm_str);
          size_t type_ssm = 0;
          type_ssm_stream >> type_ssm;
          serialized_size_max_[type_name] = type_ssm;
          RMW_CONNEXT_LOG_DEBUG_A("serialized size max: %s, %lu",
            type_name.c_str(), type_ssm)
        } else {
          RMW_CONNEXT_LOG_WARNING_A("ignored resource limits line: '%s'", line.c_str())
          continue;
        }
      }
    }
    catch(const std::exception& e)
    {
      RMW_CONNEXT_LOG_ERROR_A_SET("failed to load file: %s", e.what())
      return RMW_RET_ERROR;
    }
    
  }
  
  return RMW_RET_OK;
}

size_t
RMW_Connext_UserResourceLimits::get_serialized_size_max(const char * const type_name)
{
  auto it = serialized_size_max_.find(type_name);
  if (it == serialized_size_max_.end()) {
    return 0;
  }
  return it->second;
}
