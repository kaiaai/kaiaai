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
from kaiaai import yamledit


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
      i += 2
    else:
      rest.append(argv[i])
      i += 1
  return model, instance, rest


def _concrete(model, instance):
  # For verbs that act on ONE scope: a named model without an instance targets
  # its default, rather than leaking the active instance (e.g. "fast") across
  # models. An unnamed model (None) stays None -> the active scope.
  if model is not None and instance is None:
    instance = config.DEFAULT_INSTANCE
  return model, instance


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


def _is_active(model, instance):
  tm = config.current_model() if model is None else model
  ti = config.current_instance() if instance is None else instance
  return tm == config.current_model() and ti == config.current_instance()


def _yaml_items(scope):
  # Scoped variables whose name is "FILE.yaml/dotted.path" address a config
  # file; yield (filename, dotted_path, value) for each.
  for name in sorted(scope):
    if '/' in name:
      fname, dotted = name.split('/', 1)
      yield fname, dotted, scope[name]


def _apply_file(model, fname, dotted, value):
  try:
    path = yamledit.config_file_path(model, fname)
    yamledit.ensure_writable(path)
    was = yamledit.apply(path, dotted, value)
    print(f"  {fname}: {dotted} {was} -> {value}")
  except FileNotFoundError as err:
    print(f"  ! config file not found: {err}")
  except PermissionError as err:
    print(f"  ! read-only, cannot edit (needs a writable workspace): {err}")
  except KeyError:
    print(f"  ! path not found in {fname}: {dotted}")
  except Exception as err:
    print(f"  ! cannot resolve {model}/{fname}: {err}")


def _revert_file(model, fname, dotted):
  try:
    path = yamledit.config_file_path(model, fname)
    yamledit.ensure_writable(path)
    if yamledit.revert(path, dotted):
      print(f"  {fname}: {dotted} reverted to default")
  except FileNotFoundError as err:
    print(f"  ! config file not found: {err}")
  except PermissionError as err:
    print(f"  ! read-only, cannot edit (needs a writable workspace): {err}")
  except Exception as err:
    print(f"  ! cannot resolve {model}/{fname}: {err}")


def _render_files(model, instance, revert):
  for fname, dotted, value in _yaml_items(config.scope_vars(model=model, instance=instance)):
    if revert:
      _revert_file(model, fname, dotted)
    else:
      _apply_file(model, fname, dotted, value)


def _use(text):
  model, instance = _split_scope(text)
  # Re-render the config files: strip the scope we are leaving back to
  # defaults, then write the scope we are entering onto the untouched files.
  _render_files(config.current_model(), config.current_instance(), revert=True)
  m, i = config.use_robot(model, instance)
  _render_files(m, i, revert=False)
  print(f"now using {m}.{i}")


def _copy(src_text, dst_text):
  sm, si = _concrete(*_split_scope(src_text))
  dm, di = _concrete(*_split_scope(dst_text))
  sm = sm or config.current_model()
  dm = dm or config.current_model()
  si = si or config.current_instance()
  di = di or config.current_instance()
  copied = config.copy_scope(sm, si, dm, di)
  print(f"copied {len(copied)} var(s) {sm}.{si} -> {dm}.{di}")
  if dm == config.current_model() and di == config.current_instance():
    _render_files(dm, di, revert=False)


def _export_label(model, instance):
  if model is None:
    return "entire config"
  return model if instance is None else f"{model}.{instance}"


def _export(model, instance, rest):
  if len(rest) > 1:
    _print_usage()
    return
  text = config.dumps(config.export_config(model, instance))
  if rest:
    with open(rest[0], 'w') as file:
      file.write(text)
    print(f"exported {_export_label(model, instance)} to {rest[0]}")
  else:
    sys.stdout.write(text)


def _import(path):
  config.import_config(config.load_file(path))
  print(f"imported {path}")
  _render_files(config.current_model(), config.current_instance(), revert=False)


def _print_usage():
  print("Usage:")
  print("  kaia use MODEL[.INSTANCE]        switch the active robot (and instance)")
  print("  kaia list [--robot M[.I]]        show a scope's variables")
  print("  kaia set VAR VALUE [--robot M]   set a variable in the (given) scope")
  print("  kaia get VAR [--robot M[.I]]     print a variable")
  print("  kaia unset VAR [--robot M[.I]]   remove a variable (revert to default)")
  print("  kaia copy SRC DST                copy a scope's vars to another scope")
  print("  kaia export [--robot M[.I]] [FILE]   dump config (scope) to FILE/stdout")
  print("  kaia import FILE                 merge an exported config back in")
  print("")
  print("  VAR of the form FILE.yaml/a.b.c edits <robot_pkg>/config/FILE.yaml")
  print("  in place, e.g. navigation.yaml/amcl.ros__parameters.alpha1 0.2 ;")
  print("  unset restores the original value from its # kaia-was: comment.")
  print("")
  print("  --robot MODEL[.INSTANCE]  target another scope without switching to it")
  print("                            (.INSTANCE alone keeps the current model)")
  print("")
  print("  legacy: kaia config [VAR [VALUE]]")


def _run(argv):
  verb = argv[0]
  model, instance, rest = _pull_robot(argv[1:])

  # Verbs that span scopes parse the raw (possibly whole-model) target.
  if verb == 'use' and len(rest) == 1:
    _use(rest[0])
    return
  if verb == 'export':
    _export(model, instance, rest)
    return
  if verb == 'import' and len(rest) == 1:
    _import(rest[0])
    return
  if verb == 'copy' and len(rest) == 2:
    _copy(rest[0], rest[1])
    return

  # Remaining verbs act on a single concrete scope.
  model, instance = _concrete(model, instance)
  if verb == 'list':
    _list(model, instance)
  elif verb == 'set' and len(rest) == 2:
    name, value = rest
    config.set_var(name, value, model=model, instance=instance)
    print(f"set {name} = {value}  [{_scope_label(model, instance)}]")
    if '/' in name and _is_active(model, instance):
      fname, dotted = name.split('/', 1)
      _apply_file(config.current_model(), fname, dotted, value)
  elif verb == 'get' and len(rest) == 1:
    name = rest[0]
    if '/' in name and _is_active(model, instance):
      fname, dotted = name.split('/', 1)
      try:
        path = yamledit.config_file_path(config.current_model(), fname)
        print(yamledit.read_value(path, dotted))
      except Exception:
        print(config.get_var(name, model=model, instance=instance))
    else:
      print(config.get_var(name, model=model, instance=instance))
  elif verb == 'unset' and len(rest) == 1:
    name = rest[0]
    if config.unset_var(name, model=model, instance=instance):
      print(f"unset {name}  [{_scope_label(model, instance)}]")
      if '/' in name and _is_active(model, instance):
        fname, dotted = name.split('/', 1)
        _revert_file(config.current_model(), fname, dotted)
    else:
      print(f"{name} was not set  [{_scope_label(model, instance)}]")
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
