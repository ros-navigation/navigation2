# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the class tools utility functions."""

import inspect


def isclassinstance(obj):
    """Return True if obj is an instance of a class."""
    return hasattr(obj, '__class__')


def is_a(obj, entity_type):
    """Return True if obj is an instance of the entity_type class."""
    if not isclassinstance(obj):
        raise RuntimeError("obj '{}' is not a class instance".format(obj))
    if not inspect.isclass(entity_type):
        raise RuntimeError("entity_type '{}' is not a class".format(obj))
    return isinstance(obj, entity_type)


def is_a_subclass(obj, entity_type):
    """Return True if obj is an instance of the entity_type class or one of its subclass types."""
    if is_a(obj, entity_type):
        return True
    return issubclass(obj.__class__, entity_type)
