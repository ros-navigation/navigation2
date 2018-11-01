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

"""Module for InvalidConditionExpressionError class."""

from typing import List

from ..substitution import Substitution


class InvalidConditionExpressionError(ValueError):
    """Raised when an expression being evaluated as a condition is invalid."""

    def __init__(
        self,
        expression: str,
        unexpanded_expression: List[Substitution],
        valid_expressions: List[str]
    ) -> None:
        super().__init__(
            "invalid condition expression, expected one of [{}] but got '{}', expanded from '{}'"
            .format(
                ', '.join(valid_expressions),
                expression,
                ' + '.join([str(sub) for sub in unexpanded_expression]))
        )
