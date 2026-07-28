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


def _split_scope(text):
  # "model" -> (model, None); "model.instance" -> (model, instance);
  # ".instance" -> (None, instance) i.e. keep the current model.
  if text.startswith('.'):
    return None, text[1:] or None
  if '.' in text:
    model, instance = text.split('.', 1)
    return model, instance
  return text, None


def _pull_robot(argv):
  # Extract an optional "--robot model[.instance]" (or ".instance") from argv.
  # Returns (model, instance, remaining_argv). model/instance are None when the
  # flag is absent, meaning "operate on the active scope".
  model = instance = None
  rest = []
  i = 0
  while i < len(argv):
    if argv[i] in ('--robot', '-r') and i + 1 < len(argv):
      model, instance = _split_scope(argv[i + 1])
      # Naming another model without an instance targets its default, rather
      # than leaking the active instance (e.g. "fast") across models.
      if model is not None and instance is None:
        instance = config.DEFAULT_INSTANCE
      i += 2
    else:
      rest.append(argv[i])
      i += 1
  return model, instance, rest


def _scope_label(model, instance):
  m = config.current_model() if model is None else model
  i = config.current_instance() if instance is None else instance
  return f"{m}/{i}"


def _list(model=None, instance=None):
  m = config.current_model() if model is None else model
  i = config.current_instance() if instance is None else instance
  print(f"robot: {m}.{i}")
  scope = config.scope_vars(model=model, instance=instance)
  print(f"--- {m}/{i} ---")
  if scope:
    for name in sorted(scope):
      print(f"  {name} = {scope[name]}")
  else:
    print("  (no variables set for this scope)")
  insts = config.instances(model=m)
  if len(insts) > 1:
    print(f"instances of {m}: {', '.join(insts)}")


def _use(text):
  model, instance = _split_scope(text)
  m, i = config.use_robot(model, instance)
  print(f"now using {m}.{i}")


def _print_usage():
  print("Usage:")
  print("  kaia use MODEL[.INSTANCE]        switch the active robot (and instance)")
  print("  kaia list [--robot M[.I]]        show a scope's variables")
  print("  kaia set VAR VALUE [--robot M]   set a variable in the (given) scope")
  print("  kaia get VAR [--robot M[.I]]     print a variable")
  print("  kaia unset VAR [--robot M[.I]]   remove a variable (revert to default)")
  print("")
  print("  --robot MODEL[.INSTANCE]  target another scope without switching to it")
  print("                            (.INSTANCE alone keeps the current model)")
  print("")
  print("  legacy: kaia config [VAR [VALUE]]")


def _run(argv):
  verb = argv[0]
  model, instance, rest = _pull_robot(argv[1:])

  if verb == 'use':
    if len(rest) != 1:
      _print_usage()
      return
    _use(rest[0])
  elif verb == 'list':
    _list(model, instance)
  elif verb == 'set' and len(rest) == 2:
    config.set_var(rest[0], rest[1], model=model, instance=instance)
    print(f"set {rest[0]} = {rest[1]}  [{_scope_label(model, instance)}]")
  elif verb == 'get' and len(rest) == 1:
    print(config.get_var(rest[0], model=model, instance=instance))
  elif verb == 'unset' and len(rest) == 1:
    if config.unset_var(rest[0], model=model, instance=instance):
      print(f"unset {rest[0]}  [{_scope_label(model, instance)}]")
    else:
      print(f"{rest[0]} was not set  [{_scope_label(model, instance)}]")
  else:
    _print_usage()


def _run_legacy(argv):
  # Preserved so existing docs/scripts keep working: kaia config [VAR [VALUE]].
  count = len(argv)
  if count == 3:
    config.set_var(argv[1], argv[2])
  elif count == 2:
    print(config.get_var(argv[1]))
  elif count == 1:
    _list()


def main():
  argv = sys.argv[1:]
  if not argv:
    _print_usage()
  elif argv[0] == 'config':
    _run_legacy(argv)
  else:
    _run(argv)


if __name__ == '__main__':
  main()
