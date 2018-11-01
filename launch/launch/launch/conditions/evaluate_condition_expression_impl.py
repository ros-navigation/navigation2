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

"""Module for utility functions related to evaluating condition expressions."""

from typing import List

from .invalid_condition_expression_error import InvalidConditionExpressionError
from ..launch_context import LaunchContext
from ..substitution import Substitution
from ..utilities import perform_substitutions


VALID_TRUE_EXPRESSIONS = ['true', '1']
VALID_FALSE_EXPRESSIONS = ['false', '0']


def evaluate_condition_expression(context: LaunchContext, expression: List[Substitution]) -> bool:
    """
    Expand an expression and then evaluate it as a condition, returing true or false.

    The expanded expression is stripped and has ``lower()`` called on it before
    being logically evaluated as either true or false.
    A string will be considered True if it matches 'true' or '1'.
    A string will be considered False if it matches 'false' or '0'.
    Any other string content (including empty string) will result in an error.

    :raises: InvalidConditionExpressionError
    """
    expanded_expression = perform_substitutions(context, expression)
    expanded_expression = expanded_expression.strip().lower()
    if expanded_expression in ['true', '1']:
        return True
    if expanded_expression in ['false', '0']:
        return False
    valid_expressions = VALID_TRUE_EXPRESSIONS + VALID_FALSE_EXPRESSIONS
    raise InvalidConditionExpressionError(expanded_expression, expression, valid_expressions)
