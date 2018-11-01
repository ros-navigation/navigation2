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

"""Package for utilties."""

from .class_tools_impl import is_a, is_a_subclass, isclassinstance
from .create_future_impl import create_future
from .ensure_argument_type_impl import ensure_argument_type
from .normalize_to_list_of_substitutions_impl import normalize_to_list_of_substitutions
from .perform_substitutions_impl import perform_substitutions
from .signal_management import install_signal_handlers
from .signal_management import on_sigint
from .signal_management import on_sigquit
from .signal_management import on_sigterm
from .visit_all_entities_and_collect_futures_impl import visit_all_entities_and_collect_futures

__all__ = [
    'is_a',
    'is_a_subclass',
    'isclassinstance',
    'create_future',
    'ensure_argument_type',
    'perform_substitutions',
    'install_signal_handlers',
    'on_sigint',
    'on_sigquit',
    'on_sigterm',
    'normalize_to_list_of_substitutions',
    'visit_all_entities_and_collect_futures',
]
