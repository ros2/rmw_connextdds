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

#include "rmw_connextdds/rmw_impl.hpp"
#include "rmw_connextdds/graph_cache.hpp"

#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#if RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY
// Required to look up LOCALHOST_ONLY env variable.
#include "rcutils/get_env.h"
#endif /* RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY */

/******************************************************************************
 * Node interface functions
 ******************************************************************************/


rmw_node_t *
rmw_api_connextdds_create_node(
  rmw_context_t * context,
  const char * name,
  const char * ns
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  ,
  size_t domain_id,
  const rmw_node_security_options_t * security_options
#elif RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_ELOQUENT
  ,
  size_t domain_id,
  const rmw_node_security_options_t * security_options,
  bool localhost_only
#elif RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_FOXY
  ,
  size_t domain_id,
  bool localhost_only
#endif /* RMW_CONNEXT_RELEASE */
)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr);

  bool node_localhost_only = false;

#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_DASHING
  UNUSED_ARG(security_options);
#elif RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_ELOQUENT
  UNUSED_ARG(security_options);
  node_localhost_only = localhost_only;
#elif RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_FOXY
  node_localhost_only = localhost_only;
#else /* RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY */
  switch (context->options.localhost_only) {
    case RMW_LOCALHOST_ONLY_DEFAULT:
      {
        /* Check if user requested localhost only via environment variable */
        const char * localhost_only_env = nullptr;
        const char * lookup_rc =
          rcutils_get_env(RMW_CONNEXT_ENV_LOCALHOST_ONLY, &localhost_only_env);

        if (nullptr != lookup_rc || nullptr == localhost_only_env) {
          RMW_CONNEXT_LOG_ERROR_A_SET(
            "failed to lookup from environment: "
            "var=%s, "
            "rc=%s ",
            RMW_CONNEXT_ENV_LOCALHOST_ONLY,
            lookup_rc)
          return nullptr;
        }

        node_localhost_only = '\0' != localhost_only_env[0];
        break;
      }
    case RMW_LOCALHOST_ONLY_ENABLED:
    case RMW_LOCALHOST_ONLY_DISABLED:
      {
        node_localhost_only =
          context->options.localhost_only == RMW_LOCALHOST_ONLY_ENABLED;
        break;
      }
    default:
      {
        RMW_CONNEXT_LOG_ERROR_A_SET(
          "invalid rmw_localhost_only_t value: %d",
          context->options.localhost_only)
        return nullptr;
      }
  }
#endif /* RMW_CONNEXT_RELEASE > RMW_CONNEXT_RELEASE_FOXY */

  RMW_CONNEXT_LOG_DEBUG_A(
    "creating new node: name=%s, ns=%s, localhost_only=%d",
    name, ns, node_localhost_only)

  rmw_context_impl_t * ctx = context->impl;
  std::lock_guard<std::mutex> guard(ctx->initialization_mutex);

  // TODO(asorbini): should other (public) APIs also check whether
  // the context has already been shutdown (and fail, if so)?
  if (ctx->is_shutdown) {
    RMW_CONNEXT_LOG_ERROR_SET("context already shutdown")
    return nullptr;
  }

  /* Validate node's name and namespace */
  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    RMW_CONNEXT_LOG_ERROR_SET("failed to validate node name")
    return nullptr;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason =
      rmw_node_name_validation_result_string(validation_result);
    RMW_CONNEXT_LOG_ERROR_A_SET("invalid node name: %s", reason)
    return nullptr;
  }
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(ns, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    RMW_CONNEXT_LOG_ERROR_SET("failed to validate node namespace")
    return nullptr;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason =
      rmw_node_name_validation_result_string(validation_result);
    RMW_CONNEXT_LOG_ERROR_A_SET("invalid node namespace: %s", reason)
    return nullptr;
  }
#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_FOXY
  if (0u == ctx->node_count) {
    ctx->domain_id = domain_id;
  } else if ((size_t)ctx->domain_id != domain_id) {
    RMW_CONNEXT_LOG_ERROR_A_SET(
      "invalid domain id: context=%d, node=%ld\n",
      ctx->domain_id, domain_id)
    return nullptr;
  }
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_FOXY */

  ret = ctx->initialize_node(ns, name, node_localhost_only);
  if (RMW_RET_OK != ret) {
    RMW_CONNEXT_LOG_ERROR("failed to initialize node in context")
    return nullptr;
  }

  auto scope_exit_context_finalize = rcpputils::make_scope_exit(
    [ctx]()
    {
      if (RMW_RET_OK != ctx->finalize_node()) {
        RMW_CONNEXT_LOG_ERROR(
          "failed to finalize node in context")
      }
    });

  RMW_Connext_Node * const node_impl = RMW_Connext_Node::create(ctx);
  if (nullptr == node_impl) {
    RMW_CONNEXT_LOG_ERROR("failed to allocate node implementation")
    return nullptr;
  }

  auto scope_exit_node_impl_delete = rcpputils::make_scope_exit(
    [node_impl]()
    {
      if (RMW_RET_OK != node_impl->finalize()) {
        RMW_CONNEXT_LOG_ERROR(
          "failed to finalize node implementation")
      }
      delete node_impl;
    });

  rmw_node_t * rmw_node = rmw_node_allocate();
  if (nullptr == rmw_node) {
    RMW_CONNEXT_LOG_ERROR_SET("failed to allocate RMW node")
    return nullptr;
  }
  rmw_node->name = nullptr;
  rmw_node->namespace_ = nullptr;

  auto scope_exit_node_delete = rcpputils::make_scope_exit(
    [rmw_node]()
    {
      if (nullptr != rmw_node->name) {
        rmw_free(const_cast<char *>(rmw_node->name));
      }
      if (nullptr != rmw_node->namespace_) {
        rmw_free(const_cast<char *>(rmw_node->namespace_));
      }
      rmw_node_free(rmw_node);
    });

  const size_t name_len = strlen(name) + 1;
  rmw_node->name =
    static_cast<const char *>(
    rmw_allocate(sizeof(char) * name_len));
  if (nullptr == rmw_node->name) {
    RMW_CONNEXT_LOG_ERROR_SET("failed to allocate node name")
    return nullptr;
  }
  memcpy(const_cast<char *>(rmw_node->name), name, name_len);

  const size_t ns_len = strlen(ns) + 1;
  rmw_node->namespace_ =
    static_cast<const char *>(
    rmw_allocate(sizeof(char) * ns_len));
  if (nullptr == rmw_node->namespace_) {
    RMW_CONNEXT_LOG_ERROR_SET("failed to allocate node namespace")
    return nullptr;
  }
  memcpy(const_cast<char *>(rmw_node->namespace_), ns, ns_len);
  rmw_node->implementation_identifier = RMW_CONNEXTDDS_ID;
  rmw_node->data = node_impl;
  rmw_node->context = context;

  if (RMW_RET_OK !=
    rmw_connextdds_graph_on_node_created(ctx, rmw_node))
  {
    RMW_CONNEXT_LOG_ERROR("failed to update graph for node")
    return nullptr;
  }

  scope_exit_context_finalize.cancel();
  scope_exit_node_impl_delete.cancel();
  scope_exit_node_delete.cancel();


  RMW_CONNEXT_LOG_DEBUG_A("created new node: %p", (void *)rmw_node)

  return rmw_node;
}


rmw_ret_t
rmw_api_connextdds_destroy_node(rmw_node_t * rmw_node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    rmw_node,
    rmw_node->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CONNEXT_LOG_DEBUG_A("destroying node: %p", (void *)rmw_node)

  rmw_context_impl_t * ctx = rmw_node->context->impl;
  std::lock_guard<std::mutex> guard(ctx->initialization_mutex);

  RMW_Connext_Node * const node_impl =
    reinterpret_cast<RMW_Connext_Node *>(rmw_node->data);

  if (RMW_RET_OK !=
    rmw_connextdds_graph_on_node_deleted(ctx, rmw_node))
  {
    RMW_CONNEXT_LOG_ERROR("failed to update graph for node")
    return RMW_RET_ERROR;
  }

  rmw_free(const_cast<char *>(rmw_node->name));
  rmw_free(const_cast<char *>(rmw_node->namespace_));
  rmw_node_free(rmw_node);
  delete node_impl;

  if (RMW_RET_OK != ctx->finalize_node()) {
    RMW_CONNEXT_LOG_ERROR("failed to finalize node in context")
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}


const rmw_guard_condition_t *
rmw_api_connextdds_node_get_graph_guard_condition(const rmw_node_t * rmw_node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    rmw_node,
    rmw_node->implementation_identifier,
    RMW_CONNEXTDDS_ID,
    return nullptr);

  RMW_Connext_Node * const node_impl =
    reinterpret_cast<RMW_Connext_Node *>(rmw_node->data);

  return node_impl->graph_guard_condition();
}

#if RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_ELOQUENT

rmw_ret_t
rmw_api_connextdds_node_assert_liveliness(const rmw_node_t * node)
{
  UNUSED_ARG(node);
  return RMW_RET_UNSUPPORTED;
}
#endif /* RMW_CONNEXT_RELEASE <= RMW_CONNEXT_RELEASE_ELOQUENT */
