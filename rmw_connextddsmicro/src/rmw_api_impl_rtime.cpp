// Copyright 2020 Real-Time Innovations, Inc. (RTI)
//
// Licensed under the Apache License, Version 2.0 (the "License")
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

#include "rmw_connextdds/rmw_api_impl.hpp"

/*****************************************************************************
 * Context API
 *****************************************************************************/
const char *
rmw_get_implementation_identifier()
{
  return rmw_api_connextdds_get_implementation_identifier();
}


const char *
rmw_get_serialization_format()
{
  return rmw_api_connextdds_get_serialization_format();
}


rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  return rmw_api_connextdds_set_log_severity(severity);
}


rmw_ret_t
rmw_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator)
{
  return rmw_api_connextdds_init_options_init(init_options, allocator);
}


rmw_ret_t
rmw_init_options_copy(
  const rmw_init_options_t * src,
  rmw_init_options_t * dst)
{
  return rmw_api_connextdds_init_options_copy(src, dst);
}


rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  return rmw_api_connextdds_init_options_fini(init_options);
}


rmw_ret_t
rmw_init(
  const rmw_init_options_t * options,
  rmw_context_t * context)
{
  return rmw_api_connextdds_init(options, context);
}


rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  return rmw_api_connextdds_shutdown(context);
}


rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  return rmw_api_connextdds_context_fini(context);
}

/*****************************************************************************
 * Event API
 *****************************************************************************/

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  return rmw_api_connextdds_publisher_event_init(
    rmw_event, publisher, event_type);
}


rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  return rmw_api_connextdds_subscription_event_init(
    rmw_event, subscription, event_type);
}


rmw_ret_t
rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  return rmw_api_connextdds_take_event(event_handle, event_info, taken);
}

/*****************************************************************************
 * Info API
 *****************************************************************************/

rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  return rmw_api_connextdds_get_node_names(node, node_names, node_namespaces);
}


rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  return rmw_api_connextdds_get_node_names_with_enclaves(
    node, node_names, node_namespaces, enclaves);
}


rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * tptyp)
{
  return rmw_api_connextdds_get_topic_names_and_types(
    node, allocator, no_demangle, tptyp);
}


rmw_ret_t
rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * sntyp)
{
  return rmw_api_connextdds_get_service_names_and_types(
    node, allocator, sntyp);
}


rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  return rmw_api_connextdds_service_server_is_available(
    node, client, is_available);
}


rmw_ret_t
rmw_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  return rmw_api_connextdds_count_publishers(node, topic_name, count);
}


rmw_ret_t
rmw_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  return rmw_api_connextdds_count_subscribers(node, topic_name, count);
}


rmw_ret_t
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * tptyp)
{
  return rmw_api_connextdds_get_subscriber_names_and_types_by_node(
    node, allocator, node_name, node_namespace, no_demangle, tptyp);
}


rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * tptyp)
{
  return rmw_api_connextdds_get_publisher_names_and_types_by_node(
    node, allocator, node_name, node_namespace, no_demangle, tptyp);
}


rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * sntyp)
{
  return rmw_api_connextdds_get_service_names_and_types_by_node(
    node, allocator, node_name, node_namespace, sntyp);
}


rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * sntyp)
{
  return rmw_api_connextdds_get_client_names_and_types_by_node(
    node, allocator, node_name, node_namespace, sntyp);
}


rmw_ret_t
rmw_get_publishers_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info)
{
  return rmw_api_connextdds_get_publishers_info_by_topic(
    node, allocator, topic_name, no_mangle, publishers_info);
}


rmw_ret_t
rmw_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  return rmw_api_connextdds_get_subscriptions_info_by_topic(
    node, allocator, topic_name, no_mangle, subscriptions_info);
}

/*****************************************************************************
 * Node API
 *****************************************************************************/

rmw_node_t *
rmw_create_node(
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
)
{
  return rmw_api_connextdds_create_node(
    context, name, ns
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
    ,
    domain_id,
    security_options
#elif RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_FOXY
    ,
    domain_id,
    localhost_only
#endif /* RMW_CONNEXT_RELEASE */
  );
}



rmw_ret_t
rmw_destroy_node(rmw_node_t * rmw_node)
{
  return rmw_api_connextdds_destroy_node(rmw_node);
}


const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * rmw_node)
{
  return rmw_api_connextdds_node_get_graph_guard_condition(rmw_node);
}

#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING

rmw_ret_t
rmw_node_assert_liveliness(const rmw_node_t * node)
{
  return rmw_api_connextdds_node_assert_liveliness(node);
}
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */

/*****************************************************************************
 * Publication API
 *****************************************************************************/

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  return rmw_api_connextdds_publish(publisher, ros_message, allocation);
}


rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  return rmw_api_connextdds_publish_serialized_message(
    publisher, serialized_message, allocation);
}


rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  return rmw_api_connextdds_publish_loaned_message(
    publisher, ros_message, allocation);
}


rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  const rosidl_message_bounds_t * message_bounds,
#else
  const rosidl_runtime_c__Sequence__bound * message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
  rmw_publisher_allocation_t * allocation)
{
  return rmw_api_connextdds_init_publisher_allocation(
    type_support,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
    message_bounds,
#else
    message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
    allocation);
}


rmw_ret_t
rmw_fini_publisher_allocation(
  rmw_publisher_allocation_t * allocation)
{
  return rmw_api_connextdds_fini_publisher_allocation(allocation);
}


rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
  ,
  const rmw_publisher_options_t * publisher_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
)
{
  return rmw_api_connextdds_create_publisher(
    node, type_supports, topic_name, qos_policies
#if RMW_CONNEXT_HAVE_OPTIONS
    ,publisher_options
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  );
}


rmw_ret_t
rmw_get_gid_for_publisher(
  const rmw_publisher_t * publisher,
  rmw_gid_t * gid)
{
  return rmw_api_connextdds_get_gid_for_publisher(
    publisher, gid);
}


rmw_ret_t
rmw_compare_gids_equal(
  const rmw_gid_t * gid1,
  const rmw_gid_t * gid2,
  bool * result)
{
  return rmw_api_connextdds_compare_gids_equal(
    gid1, gid2, result);
}


rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  return rmw_api_connextdds_publisher_count_matched_subscriptions(
    publisher, subscription_count);
}


rmw_ret_t
rmw_publisher_assert_liveliness(
  const rmw_publisher_t * publisher)
{
  return rmw_api_connextdds_publisher_assert_liveliness(publisher);
}


rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  return rmw_api_connextdds_publisher_get_actual_qos(
    publisher, qos);
}


rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  return rmw_api_connextdds_borrow_loaned_message(
    publisher, type_support, ros_message);
}


rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  return rmw_api_connextdds_return_loaned_message_from_publisher(
    publisher, loaned_message);
}


rmw_ret_t
rmw_destroy_publisher(
  rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  return rmw_api_connextdds_destroy_publisher(
    node, publisher);
}

/*****************************************************************************
 * Serialization API
 *****************************************************************************/

rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * type_supports,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  const rosidl_message_bounds_t * message_bounds,
#else
  const rosidl_runtime_c__Sequence__bound * message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
  size_t * size)
{
  return rmw_api_connextdds_get_serialized_message_size(
    type_supports,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
    message_bounds,
#else
    message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
    size);
}


rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_supports,
  rmw_serialized_message_t * serialized_message)
{
  return rmw_api_connextdds_serialize(
    ros_message, type_supports, serialized_message);
}


rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_supports,
  void * ros_message)
{
  return rmw_api_connextdds_deserialize(
    serialized_message, type_supports, ros_message);
}

/*****************************************************************************
 * Service API
 *****************************************************************************/

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
#if RMW_CONNEXT_HAVE_SERVICE_INFO
  rmw_service_info_t * request_header,
#else
  rmw_request_id_t * request_header,
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */
  void * ros_response,
  bool * taken)
{
  return rmw_api_connextdds_take_response(
    client,
#if RMW_CONNEXT_HAVE_SERVICE_INFO
    request_header,
#else
    request_header,
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */
    ros_response,
    taken);
}


rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
#if RMW_CONNEXT_HAVE_SERVICE_INFO
  rmw_service_info_t * request_header,
#else
  rmw_request_id_t * request_header,
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */
  void * ros_request,
  bool * taken)
{
  return rmw_api_connextdds_take_request(
    service,
#if RMW_CONNEXT_HAVE_SERVICE_INFO
    request_header,
#else
    request_header,
#endif /* RMW_CONNEXT_HAVE_SERVICE_INFO */
    ros_request,
    taken);
}


rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_id,
  void * ros_response)
{
  return rmw_api_connextdds_send_response(service, request_id, ros_response);
}


rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  return rmw_api_connextdds_send_request(client, ros_request, sequence_id);
}


rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  return rmw_api_connextdds_create_client(
    node, type_supports, service_name, qos_policies);
}


rmw_ret_t
rmw_destroy_client(
  rmw_node_t * node,
  rmw_client_t * client)
{
  return rmw_api_connextdds_destroy_client(node, client);
}


rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  return rmw_api_connextdds_create_service(
    node, type_supports, service_name, qos_policies);
}


rmw_ret_t
rmw_destroy_service(
  rmw_node_t * node,
  rmw_service_t * service)
{
  return rmw_api_connextdds_destroy_service(node, service);
}
/*****************************************************************************
 * Subscription API
 *****************************************************************************/

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  const rosidl_message_bounds_t * message_bounds,
#else
  const rosidl_runtime_c__Sequence__bound * message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_init_subscription_allocation(
    type_support,
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
    message_bounds,
#else
    message_bounds,
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING */
    allocation);
}


rmw_ret_t
rmw_fini_subscription_allocation(
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_fini_subscription_allocation(allocation);
}


rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS
  const rmw_subscription_options_t * subscription_options
#else
  bool ignore_local_publications
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
)
{
  return rmw_api_connextdds_create_subscription(
    node, type_supports, topic_name, qos_policies,
#if RMW_CONNEXT_HAVE_OPTIONS
    subscription_options
#else
    ignore_local_publications
#endif /* RMW_CONNEXT_HAVE_OPTIONS */
  );
}



rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription, size_t * publisher_count)
{
  return rmw_api_connextdds_subscription_count_matched_publishers(
    subscription, publisher_count);
}


rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  return rmw_api_connextdds_subscription_get_actual_qos(subscription, qos);
}



rmw_ret_t
rmw_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  return rmw_api_connextdds_destroy_subscription(node, subscription);
}



rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take(
    subscription, ros_message, taken, allocation);
}


rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take_with_info(
    subscription, ros_message, taken, message_info, allocation);
}


rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take_sequence(
    subscription, count, message_sequence, message_info_sequence,
    taken, allocation);
}



rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take_serialized_message(
    subscription, serialized_message, taken, allocation);
}


rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take_serialized_message_with_info(
    subscription, serialized_message, taken, message_info, allocation);
}


rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take_loaned_message(
    subscription, loaned_message, taken, allocation);
}



rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_api_connextdds_take_loaned_message_with_info(
    subscription, loaned_message, taken, message_info, allocation);
}


rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  return rmw_api_connextdds_return_loaned_message_from_subscription(
    subscription, loaned_message);
}

/*****************************************************************************
 * WaitSet API
 *****************************************************************************/

rmw_guard_condition_t *
rmw_create_guard_condition(
  rmw_context_t * context)
{
  return rmw_api_connextdds_create_guard_condition(context);
}



rmw_ret_t
rmw_destroy_guard_condition(
  rmw_guard_condition_t * guard_condition_handle)
{
  return rmw_api_connextdds_destroy_guard_condition(guard_condition_handle);
}



rmw_ret_t
rmw_trigger_guard_condition(
  const rmw_guard_condition_t * guard_condition_handle)
{
  return rmw_api_connextdds_trigger_guard_condition(guard_condition_handle);
}



rmw_wait_set_t *
rmw_create_wait_set(
  rmw_context_t * context,
  size_t max_conditions)
{
  return rmw_api_connextdds_create_wait_set(context, max_conditions);
}



rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * rmw_ws)
{
  return rmw_api_connextdds_destroy_wait_set(rmw_ws);
}



rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subs,
  rmw_guard_conditions_t * gcs,
  rmw_services_t * srvs,
  rmw_clients_t * cls,
  rmw_events_t * evs,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  return rmw_api_connextdds_wait(
    subs, gcs, srvs, cls, evs, wait_set, wait_timeout);
}

