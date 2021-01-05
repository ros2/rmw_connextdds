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

#ifndef RMW_CONNEXTDDS__RMW_API_IMPL_HPP_
#define RMW_CONNEXTDDS__RMW_API_IMPL_HPP_

#include "rmw_connextdds/context.hpp"

/*****************************************************************************
 * Context API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
const char *
rmw_api_connextdds_get_implementation_identifier();

RMW_CONNEXTDDS_PUBLIC
const char *
rmw_api_connextdds_get_serialization_format();

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_set_log_severity(rmw_log_severity_t severity);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_init_options_copy(
  const rmw_init_options_t * src,
  rmw_init_options_t * dst);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_init_options_fini(rmw_init_options_t * init_options);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_init(
  const rmw_init_options_t * options,
  rmw_context_t * context);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_shutdown(rmw_context_t * context);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_context_fini(rmw_context_t * context);

/*****************************************************************************
 * Event API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken);

/*****************************************************************************
 * Info API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * tptyp);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * sntyp);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * tptyp);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * tptyp);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * sntyp);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * sntyp);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_publishers_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info);

/*****************************************************************************
 * Node API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_node_t *
rmw_api_connextdds_create_node(
  rmw_context_t * context,
  const char * name,
  const char * ns
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  ,
  size_t domain_id,
  const rmw_node_security_options_t * security_options
#elif RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_FOXY
  ,
  size_t domain_id,
  bool localhost_only
#endif /* RMW_CONNEXT_RELEASE */
);

RMW_CONNEXTDDS_PUBLIC

rmw_ret_t
rmw_api_connextdds_destroy_node(rmw_node_t * rmw_node);

RMW_CONNEXTDDS_PUBLIC
const rmw_guard_condition_t *
rmw_api_connextdds_node_get_graph_guard_condition(const rmw_node_t * rmw_node);

#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_node_assert_liveliness(const rmw_node_t * node);
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */

/*****************************************************************************
 * Publication API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
  rmw_api_connextdds_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  const rosidl_message_bounds_t * message_bounds,
#else
  const rosidl_runtime_c__Sequence__bound * message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
  rmw_publisher_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_fini_publisher_allocation(
  rmw_publisher_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_publisher_t *
rmw_api_connextdds_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
  ,
  const rmw_publisher_options_t * publisher_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_get_gid_for_publisher(
  const rmw_publisher_t * publisher,
  rmw_gid_t * gid);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_compare_gids_equal(
  const rmw_gid_t * gid1,
  const rmw_gid_t * gid2,
  bool * result);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publisher_assert_liveliness(
  const rmw_publisher_t * publisher);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_destroy_publisher(
  rmw_node_t * node,
  rmw_publisher_t * publisher);

/*****************************************************************************
 * Serialization API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
  rmw_api_connextdds_get_serialized_message_size(
  const rosidl_message_type_support_t * type_supports,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  const rosidl_message_bounds_t * message_bounds,
#else
  const rosidl_runtime_c__Sequence__bound * message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
  size_t * size);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_supports,
  rmw_serialized_message_t * serialized_message);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_supports,
  void * ros_message);

/*****************************************************************************
 * Service API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
  rmw_api_connextdds_take_response(
  const rmw_client_t * client,
#if RMW_CONNEXT_HAVE_SERVICE_INFO
  rmw_service_info_t * request_header,
#else
  rmw_request_id_t * request_header,
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */
  void * ros_response,
  bool * taken);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
  rmw_api_connextdds_take_request(
  const rmw_service_t * service,
#if RMW_CONNEXT_HAVE_SERVICE_INFO
  rmw_service_info_t * request_header,
#else
  rmw_request_id_t * request_header,
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */
  void * ros_request,
  bool * taken);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_id,
  void * ros_response);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id);

RMW_CONNEXTDDS_PUBLIC
rmw_client_t *
rmw_api_connextdds_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_destroy_client(
  rmw_node_t * node,
  rmw_client_t * client);

RMW_CONNEXTDDS_PUBLIC
rmw_service_t *
rmw_api_connextdds_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_destroy_service(
  rmw_node_t * node,
  rmw_service_t * service);
/*****************************************************************************
 * Subscription API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
  rmw_api_connextdds_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  const rosidl_message_bounds_t * message_bounds,
#else
  const rosidl_runtime_c__Sequence__bound * message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
  rmw_subscription_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_fini_subscription_allocation(
  rmw_subscription_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_subscription_t *
  rmw_api_connextdds_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS
  const rmw_subscription_options_t * subscription_options
#else
  bool ignore_local_publications
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  );


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription, size_t * publisher_count);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

#if RMW_CONNEXT_HAVE_TAKE_SEQ

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation);

#endif /* RMW_CONNEXT_HAVE_TAKE_SEQ */


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message);

/*****************************************************************************
 * WaitSet API
 *****************************************************************************/
RMW_CONNEXTDDS_PUBLIC
rmw_guard_condition_t *
rmw_api_connextdds_create_guard_condition(
  rmw_context_t * context);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_destroy_guard_condition(
  rmw_guard_condition_t * guard_condition_handle);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_trigger_guard_condition(
  const rmw_guard_condition_t * guard_condition_handle);


RMW_CONNEXTDDS_PUBLIC
rmw_wait_set_t *
rmw_api_connextdds_create_wait_set(
  rmw_context_t * context,
  size_t max_conditions);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_destroy_wait_set(rmw_wait_set_t * rmw_ws);


RMW_CONNEXTDDS_PUBLIC
rmw_ret_t
rmw_api_connextdds_wait(
  rmw_subscriptions_t * subs,
  rmw_guard_conditions_t * gcs,
  rmw_services_t * srvs,
  rmw_clients_t * cls,
  rmw_events_t * evs,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout);


#endif  // RMW_CONNEXTDDS__RMW_API_IMPL_HPP_
