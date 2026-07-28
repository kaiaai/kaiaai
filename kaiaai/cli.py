#!/usr/bin/env python3
#
# Copyright 2023-2024 KAIA.AI
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
import sys
from kaiaai import config


def _print_config():
  model = config.current_model()
  instance = config.current_instance()
  print(f"robot.model: {model}")
  print(f"robot.instance: {instance}")
  scope = config.scope_vars()
  print(f"--- {model} / {instance} ---")
  if scope:
    for name in sorted(scope):
      print(f"{name}: {scope[name]}")
  else:
    print("(no variables set for this model/instance)")
  insts = config.instances()
  if len(insts) > 1:
    print(f"instances of {model}: {', '.join(insts)}")


def _print_usage():
  print("Usage:")
  print("  kaia config                      list current model/instance vars")
  print("  kaia config VAR                  print a variable")
  print("  kaia config VAR VALUE            set a variable (scoped to model/instance)")
  print("  kaia config robot.model NAME     switch robot model")
  print("  kaia config robot.instance NAME  switch/create a config instance")


def main():
  argv = sys.argv[1:]
  count = len(argv)
  if count == 3 and argv[0] == "config":
    config.set_var(argv[1], argv[2])
  elif count == 2 and argv[0] == "config":
    print(config.get_var(argv[1]))
  elif count == 1 and argv[0] == "config":
    _print_config()
  elif count == 0:
    _print_usage()


if __name__ == '__main__':
  main()
