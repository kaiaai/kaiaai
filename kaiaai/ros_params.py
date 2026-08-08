#!/usr/bin/env python3
#
# Copyright 2024 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Best-effort live ROS 2 parameter push for `kaia set`.

`kaia set clean.arc_omega 0.1` persists the value (config.py) and, when a node
declaring an `arc_omega` parameter is already running, also pushes it live so
you do not have to switch to `ros2 param set`. This is deliberately best-effort:
if ROS is not sourced, no daemon is up, or no running node declares the
parameter, push() returns an empty list and the caller stays silent -- setting
config before anything is launched behaves exactly as before.

The launch convention makes this possible without a node<->section registry:
a kaia variable `SECTION.leaf` is seeded onto a node parameter named `leaf`
(see e.g. oomwoo_clean/launch/wall_clean.launch.py), so we match running nodes
by the key's leaf name and update every node that actually declares it.
"""
import os
import time

import yaml

# Short, bounded waits so the CLI never hangs on a flaky graph or a node that
# does not expose the parameter services.
GRAPH_SETTLE_SEC = 0.5
SERVICE_WAIT_SEC = 0.5
CALL_TIMEOUT_SEC = 2.0


def _full_name(name, namespace):
  ns = namespace.rstrip("/")
  return "{}/{}".format(ns, name) if ns else "/{}".format(name)


def _coerce(param_type, text):
  # Coerce the stored string to the parameter's declared type. Returns
  # (ParameterValue, None) on success or (None, reason) if it does not fit.
  from rcl_interfaces.msg import ParameterType, ParameterValue
  pv = ParameterValue()
  try:
    parsed = yaml.safe_load(text)
  except Exception:
    parsed = text
  if param_type == ParameterType.PARAMETER_BOOL:
    pv.type = ParameterType.PARAMETER_BOOL
    pv.bool_value = parsed if isinstance(parsed, bool) \
        else str(text).strip().lower() in ("true", "1", "yes", "on")
  elif param_type == ParameterType.PARAMETER_INTEGER:
    try:
      pv.type = ParameterType.PARAMETER_INTEGER
      pv.integer_value = int(parsed)
    except (TypeError, ValueError):
      return None, "'{}' is not an integer".format(text)
  elif param_type == ParameterType.PARAMETER_DOUBLE:
    try:
      pv.type = ParameterType.PARAMETER_DOUBLE
      pv.double_value = float(parsed)
    except (TypeError, ValueError):
      return None, "'{}' is not a number".format(text)
  elif param_type == ParameterType.PARAMETER_STRING:
    pv.type = ParameterType.PARAMETER_STRING
    pv.string_value = text
  else:
    return None, "unsupported parameter type (arrays not handled)"
  return pv, None


def _declared_type(node, full_name, leaf):
  # Return the parameter's declared type on `full_name`, or None if the node
  # does not declare `leaf` (or exposes no parameter service in time).
  from rcl_interfaces.msg import ParameterType
  from rcl_interfaces.srv import GetParameters
  import rclpy
  client = node.create_client(GetParameters, full_name + "/get_parameters")
  try:
    if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
      return None
    req = GetParameters.Request()
    req.names = [leaf]
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_SEC)
    resp = future.result()
    if resp is None or not resp.values:
      return None
    ptype = resp.values[0].type
    return None if ptype == ParameterType.PARAMETER_NOT_SET else ptype
  finally:
    node.destroy_client(client)


def _set(node, full_name, leaf, param_value):
  # Set leaf=param_value on full_name; returns a human-readable status string.
  from rcl_interfaces.msg import Parameter
  from rcl_interfaces.srv import SetParameters
  import rclpy
  client = node.create_client(SetParameters, full_name + "/set_parameters")
  try:
    if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
      return "no set_parameters service"
    param = Parameter()
    param.name = leaf
    param.value = param_value
    req = SetParameters.Request()
    req.parameters = [param]
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_SEC)
    resp = future.result()
    if resp is None or not resp.results:
      return "no response"
    result = resp.results[0]
    if result.successful:
      return "updated"
    return "rejected: {}".format(result.reason or "unknown")
  finally:
    node.destroy_client(client)


def push(leaf, text):
  """Push leaf=text to every running node that declares parameter `leaf`.

  Returns a list of (node_full_name, status) tuples -- empty when ROS is
  unavailable or nothing declares the parameter. Never raises.

  Set KAIA_NO_LIVE_PARAMS=1 to skip the probe entirely (useful when batch-
  setting many variables before anything is launched, so you do not pay the
  graph-discovery latency on each `kaia set`).
  """
  if os.environ.get("KAIA_NO_LIVE_PARAMS"):
    return []
  try:
    import rclpy
  except Exception:
    return []

  started = False
  node = None
  results = []
  try:
    if not rclpy.ok():
      rclpy.init()
      started = True
    node = rclpy.create_node("kaia_param_push_{}".format(os.getpid()))

    # Let the graph populate before enumerating nodes.
    deadline = time.time() + GRAPH_SETTLE_SEC
    while time.time() < deadline:
      rclpy.spin_once(node, timeout_sec=0.05)

    self_name = node.get_name()
    targets = []
    for name, namespace in node.get_node_names_and_namespaces():
      if name == self_name or name.startswith("kaia_param_push_"):
        continue
      targets.append(_full_name(name, namespace))

    for full_name in sorted(set(targets)):
      param_type = _declared_type(node, full_name, leaf)
      if param_type is None:
        continue
      param_value, reason = _coerce(param_type, text)
      if reason is not None:
        results.append((full_name, "skipped ({})".format(reason)))
        continue
      results.append((full_name, _set(node, full_name, leaf, param_value)))
  except Exception:
    # Best-effort: any ROS/runtime hiccup just means "no live push".
    return results
  finally:
    if node is not None:
      node.destroy_node()
    if started:
      try:
        rclpy.shutdown()
      except Exception:
        pass
  return results
