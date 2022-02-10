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

#include "rmw_connextdds/rmw_impl.hpp"

/******************************************************************************
 * Event Listener API
 ******************************************************************************/
rmw_ret_t
rmw_api_connextdds_event_set_callback(
  rmw_event_t * event,
  rmw_event_callback_t callback,
  const void * user_data)
{
  UNUSED_ARG(event);
  UNUSED_ARG(callback);
  UNUSED_ARG(user_data);
  RMW_CONNEXT_LOG_ERROR_SET("rmw_event_set_callback not implemented")
  return RMW_RET_UNSUPPORTED;
}

/******************************************************************************
 * Service Listener API
 ******************************************************************************/
rmw_ret_t
rmw_api_connextdds_service_set_on_new_request_callback(
  rmw_service_t * rmw_service,
  rmw_event_callback_t callback,
  const void * user_data)
{
  UNUSED_ARG(rmw_service);
  UNUSED_ARG(callback);
  UNUSED_ARG(user_data);
  RMW_CONNEXT_LOG_ERROR_SET("rmw_service_set_on_new_request_callback not implemented")
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_api_connextdds_client_set_on_new_response_callback(
  rmw_client_t * rmw_client,
  rmw_event_callback_t callback,
  const void * user_data)
{
  UNUSED_ARG(rmw_client);
  UNUSED_ARG(callback);
  UNUSED_ARG(user_data);
  RMW_CONNEXT_LOG_ERROR_SET("rmw_client_set_on_new_response_callback not implemented")
  return RMW_RET_UNSUPPORTED;
}

/******************************************************************************
 * Subscription Listener API
 ******************************************************************************/
rmw_ret_t
rmw_api_connextdds_subscription_set_on_new_message_callback(
  rmw_subscription_t * rmw_subscription,
  rmw_event_callback_t callback,
  const void * user_data)
{
  UNUSED_ARG(rmw_subscription);
  UNUSED_ARG(callback);
  UNUSED_ARG(user_data);
  RMW_CONNEXT_LOG_ERROR_SET("rmw_subscription_set_on_new_message_callback not implemented")
  return RMW_RET_UNSUPPORTED;
}
