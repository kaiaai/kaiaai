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

def main():
  argv = sys.argv[1:]
  count = len(argv)
  if count == 3 and argv[0] == "config":
    config.set_var(argv[1], argv[2])
  elif count == 2 and argv[0] == "config":
    print(config.get_var(argv[1]))
  elif count == 1 and argv[0] == "config":
    cfg = config.all_vars()
    for name in sorted(cfg):
      print(f"{name}: {cfg[name]}")
    if 'robot.model' not in cfg:
      print(f"robot.model: {config.get_var('robot.model')} (default)")
  elif count == 0:
    print("Usage:")
    print("  kaia config                   list all variables")
    print("  kaia config var_name          print a variable")
    print("  kaia config var_name value    set a variable")

if __name__ == '__main__':
  main()
