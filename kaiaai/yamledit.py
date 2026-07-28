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

# In-place, comment-preserving edits to a robot package's config YAML.
#
# Edits touch only the single target line, so the rest of a hand-tuned file
# stays byte-for-byte intact -- devs treat the YAML as the source of truth, so a
# one-line diff is the point. The pre-edit value is stashed inline as a
# "# kaia-was: <default>" comment, so unset() can restore it with no side store
# and repeated set()s keep the ORIGINAL default rather than chaining overrides.
import os
import re

MARKER = 'kaia-was:'

# A "key: value" line: (indent)(key):(everything after the colon). '.' doesn't
# match a newline, so group(3) excludes the line terminator.
_KEY_RE = re.compile(r'^(\s*)([\w.\-]+):(.*)$')
# Start of a YAML comment: a '#' at column 0 or preceded by whitespace.
_COMMENT_RE = re.compile(r'(^|\s)#')
# The trailing "# kaia-was: <default>" segment we append; always last on a line.
_WAS_RE = re.compile(r'#\s*' + re.escape(MARKER) + r'\s?(.*?)\s*$')


def config_file_path(model, filename):
  from ament_index_python.packages import get_package_share_directory
  return os.path.join(get_package_share_directory(model), 'config', filename)


def ensure_writable(path):
  if not os.path.exists(path):
    raise FileNotFoundError(path)
  if not os.access(path, os.W_OK):
    raise PermissionError(path)


def _read_lines(path):
  with open(path, 'r') as file:
    return file.readlines()


def _write_lines(path, lines):
  with open(path, 'w') as file:
    file.writelines(lines)


def _split_value_comment(after_colon):
  # Split the text after "key:" into (value, comment); comment keeps its '#'.
  match = _COMMENT_RE.search(after_colon)
  if not match:
    return after_colon.strip(), ''
  start = match.start() if match.group(1) == '' else match.start() + 1
  return after_colon[:start].strip(), after_colon[start:]


def _split_marker(comment):
  # Split a comment into (original_comment, kaia_default_or_None).
  match = _WAS_RE.search(comment)
  if not match:
    return comment.rstrip(), None
  return comment[:match.start()].rstrip(), match.group(1)


def _find_line(lines, segments):
  # Locate the line whose nesting path equals `segments` (e.g.
  # ['amcl', 'ros__parameters', 'alpha1']). Returns (index, match) or (None, None).
  stack = []
  for index, line in enumerate(lines):
    match = _KEY_RE.match(line)
    if not match:
      continue
    indent = len(match.group(1))
    while stack and stack[-1][0] >= indent:
      stack.pop()
    path = [key for _, key in stack] + [match.group(2)]
    if path == segments:
      return index, match
    stack.append((indent, match.group(2)))
  return None, None


def _rebuild(indent, key, value, comment):
  tail = ('  ' + comment) if comment else ''
  return indent + key + ': ' + str(value) + tail + '\n'


def apply(path, dotted, value):
  # Set the dotted-path leaf to `value`, stashing the prior value as a
  # kaia-was comment. Returns the captured default. Raises KeyError if the path
  # is not in the file.
  lines = _read_lines(path)
  index, match = _find_line(lines, dotted.split('.'))
  if index is None:
    raise KeyError(dotted)
  cur_value, comment = _split_value_comment(match.group(3))
  orig_comment, was = _split_marker(comment)
  if was is None:
    was = cur_value  # first edit here: the current value IS the true default
  was_segment = '# ' + MARKER + ' ' + was
  new_comment = (orig_comment + '  ' + was_segment) if orig_comment else was_segment
  lines[index] = _rebuild(match.group(1), match.group(2), value, new_comment)
  _write_lines(path, lines)
  return was


def revert(path, dotted):
  # Restore the dotted-path leaf to its kaia-was default and strip the marker.
  # Returns True if a kaia-managed line was reverted, else False.
  lines = _read_lines(path)
  index, match = _find_line(lines, dotted.split('.'))
  if index is None:
    return False
  _, comment = _split_value_comment(match.group(3))
  orig_comment, was = _split_marker(comment)
  if was is None:
    return False
  lines[index] = _rebuild(match.group(1), match.group(2), was, orig_comment)
  _write_lines(path, lines)
  return True


def read_value(path, dotted):
  # Return the current on-disk value of the dotted-path leaf, or None.
  index, match = _find_line(_read_lines(path), dotted.split('.'))
  if index is None:
    return None
  value, _ = _split_value_comment(match.group(3))
  return value
