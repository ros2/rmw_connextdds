// Copyright 2026 Real-Time Innovations, Inc. (RTI)
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

#pragma once

#include <atomic>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <dds/sub/DataReader.hpp>
#include <dds/sub/DataReaderListener.hpp>
#include <dds/sub/LoanedSamples.hpp>
#include <dds/topic/BuiltinTopic.hpp>

namespace rmw_connextdds::test
{

template<
  typename Cond,
  typename Duration1 = std::chrono::seconds,
  typename Duration2 = std::chrono::milliseconds>
bool wait_for(
  Cond condition,
  const Duration1 & timeout = Duration1(10),
  const Duration2 & interval = Duration2(100))
{
  const auto end_time = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end_time) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(interval);
  }
  return false;
}

template<typename BuiltinTopicData>
class ROSTypeFinderListener
  : public dds::sub::NoOpDataReaderListener<BuiltinTopicData>
{
public:
  explicit ROSTypeFinderListener(const std::string & topic_name)
  : topic_name_(topic_name) {}

  void on_data_available(
    dds::sub::DataReader<BuiltinTopicData> & reader) override
  {
    std::scoped_lock lock(mutex_);
    if (type_) {
      return;
    }
    process_samples(reader);
  }

  const std::optional<dds::core::xtypes::DynamicType> & get_type() const
  {
    std::scoped_lock lock(mutex_);
    return type_;
  }

private:
  void process_samples(
    dds::sub::DataReader<BuiltinTopicData> & reader)
  {
    // We only process newly seen subscribers
    auto sample_data = BuiltinTopicData();
    auto sample_info = dds::sub::SampleInfo();
    while (reader.extensions().take(sample_data, sample_info)) {
      if (!sample_info.valid() ||
        sample_data.topic_name().to_std_string() != topic_name_ ||
        !sample_data.extensions().type().has_value())
      {
        continue;
      }

      type_ = sample_data.extensions().type().value();
      reader.set_listener(nullptr);
      break;
    }
  }

  mutable std::mutex mutex_;
  std::optional<dds::core::xtypes::DynamicType> type_;
  std::string topic_name_;
};

class TypeLookupEndpointListener
  : public dds::sub::NoOpDataReaderListener<dds::topic::ParticipantBuiltinTopicData>
{
public:
  static constexpr uint32_t TYPELOOKUP_REQUEST_WRITER = 1u << 12;
  static constexpr uint32_t TYPELOOKUP_REQUEST_READER = 1u << 13;
  static constexpr uint32_t TYPELOOKUP_REPLY_WRITER = 1u << 14;
  static constexpr uint32_t TYPELOOKUP_REPLY_READER = 1u << 15;
  static constexpr uint32_t TYPELOOKUP_ALL =
    TYPELOOKUP_REQUEST_WRITER | TYPELOOKUP_REQUEST_READER |
    TYPELOOKUP_REPLY_WRITER | TYPELOOKUP_REPLY_READER;

  void on_data_available(
    dds::sub::DataReader<dds::topic::ParticipantBuiltinTopicData> & reader) override
  {
    if (found_.load()) {
      return;
    }
    for (const auto & sample : reader.take()) {
      if (!sample.info().valid()) {
        continue;
      }
      uint32_t endpoints = sample.data()->dds_builtin_endpoints();
      if ((endpoints & TYPELOOKUP_ALL) == TYPELOOKUP_ALL) {
        found_.store(true);
        return;
      }
    }
  }

  bool found() const {return found_.load();}

private:
  std::atomic_bool found_{false};
};

}  // namespace rmw_connextdds::test
