#!/usr/bin/env python
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
import yaml, os
from pathlib import Path

CONFIG_FILE_NAME = ".kaiaai.yaml"

# Variables that select the active scope; always stored at the top level. Every
# other variable is scoped under models/<robot.model>/<robot.instance>/, so a
# value set for one robot model (or instance) does not leak into another.
GLOBAL_VARS = ('robot.model', 'robot.instance')
DEFAULT_MODEL = 'makerspet_mini'
DEFAULT_INSTANCE = 'default'


def get_config_path():
  return Path.home() / CONFIG_FILE_NAME


def load():
  path = get_config_path()
  if not os.path.exists(path):
    open(path, 'a').close()

  with open(path, 'r') as file:
    config = yaml.safe_load(file)
    return {} if config is None else config


def save(config):
  with open(get_config_path(), 'w') as file:
    yaml.dump(config, file)


def current_model(config=None):
  config = load() if config is None else config
  return config.get('robot.model', DEFAULT_MODEL)


def current_instance(config=None):
  config = load() if config is None else config
  return config.get('robot.instance', DEFAULT_INSTANCE)


def _scope(config, create=False):
  model = current_model(config)
  instance = current_instance(config)
  if create:
    return config.setdefault('models', {}) \
                 .setdefault(model, {}) \
                 .setdefault(instance, {})
  return config.get('models', {}).get(model, {}).get(instance, {})


def scope_vars(config=None):
  config = load() if config is None else config
  return _scope(config)


def instances(model=None, config=None):
  config = load() if config is None else config
  model = current_model(config) if model is None else model
  return sorted(config.get('models', {}).get(model, {}).keys())


def get_var(var_name, default=None):
  config = load()
  if var_name == 'robot.model':
    return config.get('robot.model', DEFAULT_MODEL)
  if var_name == 'robot.instance':
    return config.get('robot.instance', DEFAULT_INSTANCE)
  return _scope(config).get(var_name, default)


def set_var(var_name, var_value):
  config = load()
  if var_name in GLOBAL_VARS:
    config[var_name] = var_value
  else:
    _scope(config, create=True)[var_name] = var_value
  save(config)
