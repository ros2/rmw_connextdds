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

#include <type_traits>
#include <cstdio>
#include <filesystem>

#include "test_utils.hpp"

#include "gtest/gtest.h"

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "test_msgs/msg/keyed_long.hpp"
#include "test_msgs/msg/keyed_string.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include <dds/core/xtypes/DynamicData.hpp>
#include <dds/dds.hpp>
#include <rti/rti.hpp>

namespace
{
constexpr auto ros_node_name = "test_node";
constexpr auto ros_node_namespace = "/test_namespace";
constexpr auto ros_topic_name = "/test";

constexpr auto connext_topic_name = "rt/test";

template<typename T>
dds::core::xtypes::DynamicData to_dynamic_data(
  const T & message,
  const dds::core::xtypes::DynamicType & type);

template<>
dds::core::xtypes::DynamicData to_dynamic_data(
  const test_msgs::msg::KeyedString & message,
  const dds::core::xtypes::DynamicType & type)
{
  auto dds_data = dds::core::xtypes::DynamicData(type);
  dds_data.value("key", message.key);
  dds_data.value("value", message.value);
  return dds_data;
}

template<>
dds::core::xtypes::DynamicData to_dynamic_data(
  const test_msgs::msg::KeyedLong & message,
  const dds::core::xtypes::DynamicType & type)
{
  auto dds_data = dds::core::xtypes::DynamicData(type);
  dds_data.value("key", message.key);
  dds_data.value("value", message.value);
  return dds_data;
}

template<typename T>
auto get_messages()
{
  if constexpr (std::is_same_v<T, test_msgs::msg::KeyedString>) {
    return get_messages_keyed_string();
  } else if constexpr (std::is_same_v<T, test_msgs::msg::KeyedLong>) {
    return get_messages_keyed_long();
  } else {
    static_assert(false, "Unsupported type");
  }
}

}  // namespace

template<typename TestType>
class TestKeyedTopics : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto options = rmw_get_zero_initialized_init_options();
    auto ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      auto ret = rmw_init_options_fini(&options);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    });
    options.enclave = rcutils_strdup("/", rcutils_get_default_allocator());
    ASSERT_STREQ("/", options.enclave);
    ret = rmw_init(&options, &context);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    node = rmw_create_node(&context, ros_node_name, ros_node_namespace);
    ASSERT_NE(nullptr, node) << rmw_get_error_string().str;
  }

  void TearDown() override
  {
    auto ret = rmw_destroy_node(node);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_shutdown(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_context_fini(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;

    // Explicitly finalize the participant factory to avoid invalid memory
    // access errors when running multiple tests in the same process.
    dds::domain::DomainParticipant::finalize_participant_factory();
  }

  rmw_context_t context{rmw_get_zero_initialized_context()};
  rmw_node_t *node{nullptr};
};

using KeyedTypes = ::testing::Types<
  test_msgs::msg::KeyedLong,
  test_msgs::msg::KeyedString>;

struct KeyedTypesNameGenerator
{
  template<typename T>
  static std::string GetName([[maybe_unused]] int idx)
  {
    if constexpr (std::is_same_v<T, test_msgs::msg::KeyedLong>) {
      return "KeyedLong";
    }
    if constexpr (std::is_same_v<T, test_msgs::msg::KeyedString>) {
      return "KeyedString";
    }

    return "";
  }
};

TYPED_TEST_SUITE(TestKeyedTopics, KeyedTypes, KeyedTypesNameGenerator);

// Create a ROS publisher and subscriber.
// Create a DDS DataWriter and DataReader.
// Publish with ROS two samples from different instances.
// Make sure we receive the samples with ROS.
// Read the samples with Connext:
// - Take the instance handle of both samples and convert them to GUID
// - Check that GUID has the expected value:
//   - For that, we publish two samples with DDS.
//   - Check the instance handles received from both ROS and DDS are equal.
TYPED_TEST(TestKeyedTopics, test_keyhash)
{
  const auto * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<TypeParam>();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  // Create RMW publisher
  auto publisher_options = rmw_get_default_publisher_options();
  rmw_publisher_t *pub = rmw_create_publisher(
    this->node,
    ts,
    ros_topic_name,
    &qos_profile,
    &publisher_options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(this->node, pub)) << rmw_get_error_string().str;
  );

  // Create RMW subscriber
  auto subscription_options = rmw_get_default_subscription_options();
  rmw_subscription_t *sub = rmw_create_subscription(
    this->node,
    ts,
    ros_topic_name,
    &qos_profile,
    &subscription_options);
  ASSERT_NE(nullptr, sub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(this->node, sub)) << rmw_get_error_string().str;
  );

  // Configure the participant factory to manually enable entities
  auto factory_qos = dds::domain::DomainParticipant::participant_factory_qos();
  factory_qos << dds::core::policy::EntityFactory::ManuallyEnable();
  dds::domain::DomainParticipant::participant_factory_qos(factory_qos);

  // Create Domain Participant
  auto participant = dds::domain::DomainParticipant(this->context.actual_domain_id);

  // Create shared pointer to BuiltinParticipantListener class
  auto publication_listener =
    std::make_shared<rmw_connextdds::test::ROSTypeFinderListener>(connext_topic_name);

  // Get builtin subscriber's datareader for publications.
  auto publication_reader =
    std::vector<dds::sub::DataReader<dds::topic::PublicationBuiltinTopicData>>();
  dds::sub::find<dds::sub::DataReader<dds::topic::PublicationBuiltinTopicData>>(
    dds::sub::builtin_subscriber(participant),
    dds::topic::publication_topic_name(),
    std::back_inserter(publication_reader));

  publication_reader.front().set_listener(publication_listener);
  participant.enable();

  ASSERT_TRUE(
    rmw_connextdds::test::wait_for(
      [&publication_listener]
      {
        return std::nullopt != publication_listener->get_type();
      }));

  auto topic = dds::topic::Topic<dds::core::xtypes::DynamicData>(
    participant,
    connext_topic_name,
    *publication_listener->get_type());

  // Create a Reliable DataWriter
  auto writer_qos = dds::core::QosProvider::Default().datawriter_qos();
  writer_qos << dds::core::policy::Reliability::Reliable();
  writer_qos << dds::core::policy::Durability::TransientLocal();
  auto writer = dds::pub::DataWriter<dds::core::xtypes::DynamicData>(topic, writer_qos);

  // Create a Reliable DataReader
  auto reader_qos = dds::core::QosProvider::Default().datareader_qos();
  reader_qos << dds::core::policy::Reliability::Reliable();
  reader_qos << dds::core::policy::Durability::TransientLocal();
  reader_qos << rti::core::policy::Property().set({
    "dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size",
    "8192"
  });
  auto reader = dds::sub::DataReader<dds::core::xtypes::DynamicData>(topic, reader_qos);

  // Create RMW messages to be published
  const auto rmw_message_instances = get_messages<TypeParam>();
  // Publish the RMW messages
  for (const auto & rmw_message_instance : rmw_message_instances) {
    const auto ret = rmw_publish(pub, rmw_message_instance.get(), nullptr);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  }

  // Read ROS messages using ROS
  auto rmw_received_messages = decltype(rmw_message_instances)();

  // Take RMW samples
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&]{
      auto rmw_output_message = TypeParam();
      auto taken = false;
      const auto ret = rmw_take(sub, &rmw_output_message, &taken, nullptr);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;

      if (taken) {
        rmw_received_messages.push_back(
          std::make_shared<TypeParam>(std::move(rmw_output_message)));
      }

      return rmw_message_instances.size() == rmw_received_messages.size();
    }));

  ASSERT_TRUE(
    std::is_permutation(
      rmw_received_messages.begin(),
      rmw_received_messages.end(),
      rmw_message_instances.begin(),
      [](const auto & lhs, const auto & rhs) {
        return *lhs == *rhs;
      }));

  // Create DDS messages to be published
  auto dds_message_instances = std::vector<dds::core::xtypes::DynamicData>();
  for (const auto & rmw_message_instance : rmw_message_instances) {
    dds_message_instances.push_back(
      to_dynamic_data(*rmw_message_instance, *publication_listener->get_type()));
  }

  // Read ROS messages using DDS
  auto dds_received_messages_from_ros = std::pair<
    std::vector<dds::core::InstanceHandle>,
    std::vector<dds::core::xtypes::DynamicData>>();

  // Take RMW samples data and instance handles
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for(
      [&] {
        auto sample_data = dds::core::xtypes::DynamicData(*publication_listener->get_type());
        auto sample_info = dds::sub::SampleInfo();
        while (reader.extensions().take(sample_data, sample_info)) {
          if (sample_info.valid()) {
            dds_received_messages_from_ros.first.push_back(sample_info.instance_handle());
            dds_received_messages_from_ros.second.push_back(sample_data);
          }
        }
        return dds_message_instances.size() == dds_received_messages_from_ros.second.size();
    }));

  ASSERT_TRUE(std::is_permutation(
    dds_received_messages_from_ros.second.begin(),
    dds_received_messages_from_ros.second.end(),
    dds_message_instances.begin()));

  // Publish the DDS messages
  for (const auto & dds_message_instance : dds_message_instances) {
    writer.write(dds_message_instance);
  }

  // Read DDS messages using DDS
  auto dds_received_messages_from_dds = std::pair<
    std::vector<dds::core::InstanceHandle>,
    std::vector<dds::core::xtypes::DynamicData>>();

  // Take DDS samples data and instance handles
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for(
      [&] {
        auto sample_data = dds::core::xtypes::DynamicData(*publication_listener->get_type());
        auto sample_info = dds::sub::SampleInfo();
        while (reader.extensions().take(sample_data, sample_info)) {
          if (sample_info.valid()) {
            dds_received_messages_from_dds.first.push_back(sample_info.instance_handle());
            dds_received_messages_from_dds.second.push_back(sample_data);
          }
        }
        return dds_message_instances.size() == dds_received_messages_from_dds.second.size();
    }));

  ASSERT_TRUE(std::is_permutation(
    dds_received_messages_from_dds.second.begin(),
    dds_received_messages_from_dds.second.end(),
    dds_message_instances.begin()));

  // Check that the instance handles are the same
  ASSERT_EQ(dds_received_messages_from_ros.first, dds_received_messages_from_dds.first);

  // Read DDS samples with ROS
  rmw_received_messages.clear();
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&]{
      auto rmw_output_message = TypeParam();
      auto taken = false;
      const auto ret = rmw_take(sub, &rmw_output_message, &taken, nullptr);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;

      if (taken) {
        rmw_received_messages.push_back(
          std::make_shared<TypeParam>(std::move(rmw_output_message)));
      }

      return rmw_message_instances.size() == rmw_received_messages.size();
    }));

  ASSERT_TRUE(
    std::is_permutation(
      rmw_received_messages.begin(),
      rmw_received_messages.end(),
      rmw_message_instances.begin(),
      [](const auto & lhs, const auto & rhs) {
        return *lhs == *rhs;
      }));
}
