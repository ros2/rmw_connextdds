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

#include <chrono>
#include <memory>
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

class ROSTypeFinderListener
  : public dds::sub::NoOpDataReaderListener<
    dds::topic::PublicationBuiltinTopicData>
{
public:
  explicit ROSTypeFinderListener(const std::string & topic_name)
  : topic_name_(topic_name) {}
  // This gets called when a subscriber has been discovered
  void on_data_available(
    dds::sub::DataReader<dds::topic::PublicationBuiltinTopicData> & reader) override
  {
    // We only process newly seen subscribers
    dds::sub::LoanedSamples<dds::topic::PublicationBuiltinTopicData> samples =
      reader.select().state(dds::sub::status::DataState::new_instance()).take();

    for (const auto & sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }

      if (sample.data().topic_name().to_std_string() != topic_name_) {
        continue;
      }

      if (!sample.data()->get_type_no_copy().has_value()) {
        continue;
      }

      type_ = std::make_unique<dds::core::xtypes::DynamicType>(sample.data()->type().value());
    }
  }

  const std::unique_ptr<dds::core::xtypes::DynamicType> & get_type() const
  {
    return type_;
  }

private:
  std::unique_ptr<dds::core::xtypes::DynamicType> type_;
  std::string topic_name_;
};

}  // namespace rmw_connextdds::test
