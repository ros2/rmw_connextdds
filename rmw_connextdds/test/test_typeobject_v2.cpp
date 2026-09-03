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

// Test TypeObject V2 type compatibility between evolved types.
//
// Each test runs independently with fresh participants to avoid TypeObject caching.
// The four combinations test RMW and Connext endpoints:
// - "RMW" endpoint: created via rmw_create_publisher/rmw_create_subscription
// - "Connext" endpoint: DDS entity on a standalone participant with an evolved type
//
// The Connext side discovers the RMW type via built-in topics, then builds
// a superset or subset dynamic type to test TypeObject V2 assignability.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "test_utils.hpp"

#include "gtest/gtest.h"

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "test_msgs/msg/keyed_long.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"
#include "rcutils/env.h"

#include <dds/core/xtypes/DynamicData.hpp>
#include <dds/dds.hpp>
#include <rti/rti.hpp>

namespace
{

constexpr auto ros_node_name = "test_node";
constexpr auto ros_node_namespace = "/test_namespace";
constexpr auto ros_topic_name = "/test_typeobject_v2";
constexpr auto connext_topic_name = "rt/test_typeobject_v2";
constexpr auto num_samples = std::size_t{3};

// Reader type: struct A { long x; } with mutable extensibility (for pure DDS test)
dds::core::xtypes::StructType make_reader_type()
{
  auto type = dds::core::xtypes::StructType("A");
  type.extensibility_kind(dds::core::xtypes::ExtensibilityKind::MUTABLE);
  type.add_member(dds::core::xtypes::Member("x", dds::core::xtypes::primitive_type<int32_t>()));
  return type;
}

// Writer type: struct A { long x; long y; } with mutable extensibility (for pure DDS test)
dds::core::xtypes::StructType make_writer_type()
{
  auto type = dds::core::xtypes::StructType("A");
  type.extensibility_kind(dds::core::xtypes::ExtensibilityKind::MUTABLE);
  type.add_member(dds::core::xtypes::Member("x", dds::core::xtypes::primitive_type<int32_t>()));
  type.add_member(dds::core::xtypes::Member("y", dds::core::xtypes::primitive_type<int32_t>()));
  return type;
}

// Build a superset type from the discovered type by appending an extra int32 field
dds::core::xtypes::StructType make_superset_type(
  const dds::core::xtypes::StructType & base)
{
  auto type = dds::core::xtypes::StructType(base.name());
  type.extensibility_kind(base.extensibility_kind());
  for (uint32_t i = 0; i < base.member_count(); ++i) {
    type.add_member(base.member(i));
  }
  type.add_member(
    dds::core::xtypes::Member("extra_field", dds::core::xtypes::primitive_type<int32_t>()));
  return type;
}

// Build a subset type from the discovered type by keeping only the first field
dds::core::xtypes::StructType make_subset_type(
  const dds::core::xtypes::StructType & base)
{
  auto type = dds::core::xtypes::StructType(base.name());
  type.extensibility_kind(base.extensibility_kind());
  type.add_member(base.member(0));
  return type;
}

}  // namespace

// Each test gets a fresh RMW context (and thus a fresh DDS participant) to avoid
// TypeObject state caching between tests.
class TypeObjectV2Test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Use the "never" participant QoS override policy so the RMW does not force
    // TypeObject V1 length or channel restrictions. Use NDDS_QOS_PROFILES to
    // explicitly enable TypeObject V2: LENGTH_AUTO for
    // type_object_max_serialized_length and the TypeLookup Service builtin channel
    // for enabled_builtin_channels.
    ASSERT_TRUE(rcutils_set_env("RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY", "never"));
    ASSERT_TRUE(rcutils_set_env(
      "NDDS_QOS_PROFILES",
      "str://\"<dds>"
        "<qos_library name=\"TestLib\">"
          "<qos_profile name=\"TestProfile\""
            " is_default_participant_factory_profile=\"true\">"
            "<domain_participant_qos>"
              "<resource_limits>"
                "<type_object_max_serialized_length>LENGTH_AUTO</type_object_max_serialized_length>"
              "</resource_limits>"
              "<discovery_config>"
                "<enabled_builtin_channels>"
                  "DDS_DISCOVERYCONFIG_TYPE_LOOKUP_SERVICE_CHANNEL"
                "</enabled_builtin_channels>"
              "</discovery_config>"
            "</domain_participant_qos>"
          "</qos_profile>"
        "</qos_library>"
      "</dds>\""));

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
    ret = rmw_init(&options, &context_);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    node_ = rmw_create_node(&context_, ros_node_name, ros_node_namespace);
    ASSERT_NE(nullptr, node_) << rmw_get_error_string().str;
  }

  void TearDown() override
  {
    // Destroy connext_participant_ before RMW shutdown: rmw_context_fini tears
    // down global DDS infrastructure (including the participant factory), so the
    // standalone participant must be gone before that happens or its destructor
    // will crash trying to access the already-invalidated native state.
    connext_participant_ = dds::domain::DomainParticipant(dds::core::null);
    auto ret = rmw_destroy_node(node_);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_shutdown(&context_);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_context_fini(&context_);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ASSERT_TRUE(rcutils_set_env("NDDS_QOS_PROFILES", NULL));
    ASSERT_TRUE(rcutils_set_env("RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY", NULL));
    dds::domain::DomainParticipant::finalize_participant_factory();
  }

  // Create a standalone Connext DDS participant (the "Connext" side).
  // The participant is returned disabled so callers can install listeners on
  // built-in topic readers before calling enable(), guaranteeing no discovery
  // data is missed.
  dds::domain::DomainParticipant create_connext_participant()
  {
    auto factory_qos = dds::domain::DomainParticipant::participant_factory_qos();
    factory_qos << dds::core::policy::EntityFactory::ManuallyEnable();
    dds::domain::DomainParticipant::participant_factory_qos(factory_qos);
    auto participant_qos = dds::core::QosProvider::Default().participant_qos();
    participant_qos << rti::core::policy::TransportBuiltin(
      rti::core::policy::TransportBuiltinMask::udpv4());
    auto discovery_config =
      participant_qos.policy<rti::core::policy::DiscoveryConfig>();
    discovery_config.request_types_filter("*");
    participant_qos << discovery_config;
    auto participant = dds::domain::DomainParticipant(
      context_.actual_domain_id, participant_qos);

    return participant;
  }

  // Wait for a ROSTypeFinderListener to discover the RMW type.
  template<typename BuiltinTopicData>
  dds::core::xtypes::StructType discover_rmw_type(
    std::shared_ptr<rmw_connextdds::test::ROSTypeFinderListener<BuiltinTopicData>> listener)
  {
    bool found = rmw_connextdds::test::wait_for([&] {
          return listener->get_type().has_value();
      });
    EXPECT_TRUE(found) << "Failed to discover RMW type via built-in topics";
    if (!found) {
      return dds::core::xtypes::StructType("");
    }
    auto discovered_type = listener->get_type();
    return static_cast<const dds::core::xtypes::StructType &>(*discovered_type);
  }

  // Verify that at least one remotely discovered participant advertises
  // TypeLookup Service builtin endpoints (proves TypeObject V2 is active).
  // The listener must have been attached to the participant built-in topic
  // reader before the Connext participant was enabled.
  void assert_remote_typelookup_endpoints(
    std::shared_ptr<rmw_connextdds::test::TypeLookupEndpointListener> listener)
  {
    bool found = rmw_connextdds::test::wait_for([&] {
          return listener->found();
      });
    ASSERT_TRUE(found)
      << "No remote participant found with TypeLookup Service endpoints";
  }

  // Set up a Connext participant with built-in topic listeners to discover
  // the RMW endpoint type via TypeObject V2. Returns true if setup and
  // discovery succeeded.
  template<typename BuiltinTopicData>
  bool create_connext_participant_and_discover_type(
    const std::string & endpoint_topic_name,
    dds::core::xtypes::StructType & out_base_type)
  {
    auto listener = std::make_shared<
      rmw_connextdds::test::ROSTypeFinderListener<BuiltinTopicData>>(connext_topic_name);
    auto typelookup_listener =
      std::make_shared<rmw_connextdds::test::TypeLookupEndpointListener>();
    connext_participant_ = create_connext_participant();
    auto endpoint_reader =
      rti::sub::find_datareader_by_topic_name<dds::sub::DataReader<BuiltinTopicData>>(
        dds::sub::builtin_subscriber(connext_participant_), endpoint_topic_name);
    if (endpoint_reader == dds::core::null) {
      ADD_FAILURE() << "No endpoint built-in topic reader found for " << endpoint_topic_name;
      return false;
    }
    endpoint_reader.set_listener(listener);
    auto participant_reader =
      rti::sub::find_datareader_by_topic_name<
        dds::sub::DataReader<dds::topic::ParticipantBuiltinTopicData>>(
          dds::sub::builtin_subscriber(connext_participant_),
          dds::topic::participant_topic_name());
    if (participant_reader == dds::core::null) {
      ADD_FAILURE() << "No participant built-in topic reader found";
      return false;
    }
    participant_reader.set_listener(typelookup_listener);
    connext_participant_.enable();
    out_base_type = discover_rmw_type(listener);
    assert_remote_typelookup_endpoints(typelookup_listener);
    return !HasFatalFailure();
  }

  rmw_context_t context_{rmw_get_zero_initialized_context()};
  rmw_node_t * node_{nullptr};
  dds::domain::DomainParticipant connext_participant_{dds::core::null};
};

// Connext Writer (writer_type) -> Connext Reader (reader_type)
// Both endpoints on separate standalone Connext participants using MUTABLE dynamic types.
TEST_F(TypeObjectV2Test, connext_writer_connext_reader)
{
  auto writer_participant = create_connext_participant();
  auto reader_participant = create_connext_participant();
  writer_participant.enable();
  reader_participant.enable();

  auto wtype = make_writer_type();
  auto rtype = make_reader_type();

  auto writer_topic = dds::topic::Topic<dds::core::xtypes::DynamicData>(
    writer_participant, connext_topic_name, wtype);
  auto reader_topic = dds::topic::Topic<dds::core::xtypes::DynamicData>(
    reader_participant, connext_topic_name, rtype);

  auto writer_qos = dds::core::QosProvider::Default().datawriter_qos();
  writer_qos << dds::core::policy::Reliability::Reliable();
  writer_qos << dds::core::policy::Durability::TransientLocal();
  writer_qos << dds::core::policy::History::KeepAll();
  auto writer = dds::pub::DataWriter<dds::core::xtypes::DynamicData>(writer_topic, writer_qos);

  auto reader_qos = dds::core::QosProvider::Default().datareader_qos();
  reader_qos << dds::core::policy::Reliability::Reliable();
  reader_qos << dds::core::policy::Durability::TransientLocal();
  reader_qos << dds::core::policy::History::KeepAll();
  auto reader = dds::sub::DataReader<dds::core::xtypes::DynamicData>(reader_topic, reader_qos);

  // Wait for discovery
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      return writer.publication_matched_status().current_count() > 0 &&
             reader.subscription_matched_status().current_count() > 0;
    })) << "Writer and reader did not discover each other";

  // Write samples
  for (int32_t i = 0; i < num_samples; ++i) {
    auto sample = dds::core::xtypes::DynamicData(wtype);
    sample.value("x", i + 1);
    sample.value("y", (i + 1) * 10);
    writer.write(sample);
  }

  // Read and verify
  std::vector<dds::core::xtypes::DynamicData> received;
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      auto sample_data = dds::core::xtypes::DynamicData(rtype);
      auto sample_info = dds::sub::SampleInfo();
      while (reader.extensions().take(sample_data, sample_info)) {
        if (sample_info.valid()) {
          received.push_back(sample_data);
        }
      }
      return received.size() == num_samples;
    })) << "Reader did not receive all samples, received "
        << received.size() << " of " << num_samples;

  for (int32_t i = 0; i < num_samples; ++i) {
    EXPECT_EQ(received[i].value<int32_t>("x"), i + 1);
  }
}

// Connext Writer (superset type) -> RMW Reader (test_msgs/msg/KeyedLong)
// Connext writes with extra fields; RMW reader ignores them via TypeObject V2.
TEST_F(TypeObjectV2Test, connext_writer_rmw_reader)
{
  const auto * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::KeyedLong>();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  auto subscription_options = rmw_get_default_subscription_options();
  auto * sub = rmw_create_subscription(
    node_, ts, ros_topic_name, &qos_profile, &subscription_options);
  ASSERT_NE(nullptr, sub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, sub)) << rmw_get_error_string().str;
  );

  // Create Connext participant and discover the RMW type (via subscription built-in topic)
  dds::core::xtypes::StructType base_type("");
  ASSERT_TRUE(
    create_connext_participant_and_discover_type<dds::topic::SubscriptionBuiltinTopicData>(
      dds::topic::subscription_topic_name(), base_type));

  // Build superset type (base + extra_field)
  auto superset_type = make_superset_type(base_type);

  auto topic = dds::topic::Topic<dds::core::xtypes::DynamicData>(
    connext_participant_, connext_topic_name, superset_type);

  auto writer_qos = dds::core::QosProvider::Default().datawriter_qos();
  writer_qos << dds::core::policy::Reliability::Reliable();
  writer_qos << dds::core::policy::Durability::TransientLocal();
  writer_qos << dds::core::policy::History::KeepAll();
  auto writer = dds::pub::DataWriter<dds::core::xtypes::DynamicData>(topic, writer_qos);

  // Wait for Connext writer to match the RMW reader
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      return writer.publication_matched_status().current_count() > 0;
    })) << "Connext writer did not match RMW reader";

  // Write samples with superset type
  for (int32_t i = 0; i < num_samples; ++i) {
    auto sample = dds::core::xtypes::DynamicData(superset_type);
    sample.value("key", i + 1);
    sample.value("value", (i + 1) * 10);
    sample.value("extra_field", static_cast<int32_t>(999));
    writer.write(sample);
  }

  // Take via RMW and verify (extra_field should be ignored)
  std::vector<test_msgs::msg::KeyedLong> received;
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      test_msgs::msg::KeyedLong msg;
      bool taken = false;
      auto ret = rmw_take(sub, &msg, &taken, nullptr);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
      if (taken) {
        received.push_back(msg);
      }
      return received.size() == num_samples;
    })) << "RMW reader did not receive all samples, received "
        << received.size() << " of " << num_samples;

  for (int32_t i = 0; i < num_samples; ++i) {
    EXPECT_EQ(received[i].key, i + 1);
    EXPECT_EQ(received[i].value, (i + 1) * 10);
  }
}

// RMW Writer (test_msgs/msg/KeyedLong) -> Connext Reader (subset type)
// Connext reader has fewer fields; TypeObject V2 allows ignoring missing fields.
TEST_F(TypeObjectV2Test, rmw_writer_connext_reader)
{
  const auto * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::KeyedLong>();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  // Create RMW publisher
  auto publisher_options = rmw_get_default_publisher_options();
  auto * pub = rmw_create_publisher(
    node_, ts, ros_topic_name, &qos_profile, &publisher_options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, pub)) << rmw_get_error_string().str;
  );

  // Create Connext participant and discover the RMW type (via publication built-in topic)
  dds::core::xtypes::StructType base_type("");
  ASSERT_TRUE(
    create_connext_participant_and_discover_type<dds::topic::PublicationBuiltinTopicData>(
      dds::topic::publication_topic_name(), base_type));

  // Build subset type (first field only)
  ASSERT_GT(base_type.member_count(), 0u) << "Discovered base type has no members";
  auto subset_type = make_subset_type(base_type);

  auto topic = dds::topic::Topic<dds::core::xtypes::DynamicData>(
    connext_participant_, connext_topic_name, subset_type);

  auto reader_qos = dds::core::QosProvider::Default().datareader_qos();
  reader_qos << dds::core::policy::Reliability::Reliable();
  reader_qos << dds::core::policy::Durability::TransientLocal();
  reader_qos << dds::core::policy::History::KeepAll();
  auto reader = dds::sub::DataReader<dds::core::xtypes::DynamicData>(topic, reader_qos);

  // Wait for RMW writer to match Connext reader
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      return reader.subscription_matched_status().current_count() > 0;
    })) << "RMW writer and Connext reader did not match";

  // Publish via RMW
  for (int32_t i = 0; i < num_samples; ++i) {
    test_msgs::msg::KeyedLong msg;
    msg.key = i + 1;
    msg.value = (i + 1) * 10;
    auto ret = rmw_publish(pub, &msg, nullptr);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  }

  // Take with Connext reader (subset - only first field)
  std::vector<dds::core::xtypes::DynamicData> received;
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      auto sample_data = dds::core::xtypes::DynamicData(subset_type);
      auto sample_info = dds::sub::SampleInfo();
      while (reader.extensions().take(sample_data, sample_info)) {
        if (sample_info.valid()) {
          received.push_back(sample_data);
        }
      }
      return received.size() == num_samples;
    })) << "Connext reader did not receive all samples, received "
        << received.size() << " of " << num_samples;

  for (int32_t i = 0; i < num_samples; ++i) {
    EXPECT_EQ(received[i].value<int32_t>(base_type.member(0).name()), i + 1);
  }
}

// RMW Writer -> RMW Reader (both test_msgs/msg/KeyedLong)
// Verifies standard RMW pub/sub works on the test fixture's participant.
TEST_F(TypeObjectV2Test, rmw_writer_rmw_reader)
{
  const auto * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::KeyedLong>();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  // Create RMW publisher
  auto publisher_options = rmw_get_default_publisher_options();
  auto * pub = rmw_create_publisher(
    node_, ts, ros_topic_name, &qos_profile, &publisher_options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, pub)) << rmw_get_error_string().str;
  );

  // Create RMW subscriber
  auto subscription_options = rmw_get_default_subscription_options();
  auto * sub = rmw_create_subscription(
    node_, ts, ros_topic_name, &qos_profile, &subscription_options);
  ASSERT_NE(nullptr, sub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, sub)) << rmw_get_error_string().str;
  );

  // Wait for the RMW subscription to report a matched publisher.
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      size_t count = 0;
      rmw_ret_t ret = rmw_subscription_count_matched_publishers(sub, &count);
      return ret == RMW_RET_OK && count > 0;
    })) << "RMW writer and reader did not match";

  // Publish via RMW
  for (int32_t i = 0; i < num_samples; ++i) {
    test_msgs::msg::KeyedLong msg;
    msg.key = i + 1;
    msg.value = (i + 1) * 10;
    auto ret = rmw_publish(pub, &msg, nullptr);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  }

  // Take via RMW
  std::vector<test_msgs::msg::KeyedLong> received;
  ASSERT_TRUE(
    rmw_connextdds::test::wait_for([&] {
      test_msgs::msg::KeyedLong msg;
      bool taken = false;
      auto ret = rmw_take(sub, &msg, &taken, nullptr);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
      if (taken) {
        received.push_back(msg);
      }
      return received.size() == num_samples;
    })) << "RMW reader did not receive all samples, received "
        << received.size() << " of " << num_samples;

  for (int32_t i = 0; i < num_samples; ++i) {
    EXPECT_EQ(received[i].key, i + 1);
    EXPECT_EQ(received[i].value, (i + 1) * 10);
  }
}
