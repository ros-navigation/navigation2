# Copyright (c) 2019 Intel Corporation
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

from collections.abc import Generator
import tempfile
from typing import Optional, TypeAlias, Union

import launch
import yaml

YamlValue: TypeAlias = Union[str, int, float, bool]


class DictItemReference:

    def __init__(self, dictionary: dict[str, YamlValue], key: str):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self) -> str:
        return self.dictKey

    def setValue(self, value: YamlValue) -> None:
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):  # type: ignore[misc]
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: dict[str, launch.SomeSubstitutionsType],
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        convert_types: bool = False,
    ) -> None:
        super().__init__()
        """
        Construct the substitution

        :param: source_file the original YAML file to modify
        :param: param_rewrites mappings to replace
        :param: root_key if provided, the contents are placed under this key
        :param: key_rewrites keys of mappings to replace
        :param: convert_types whether to attempt converting the string to a number or boolean
        """

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions

        self.__source_file: list[launch.Substitution] = \
            normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {}
        self.__key_rewrites = {}
        self.__convert_types = convert_types
        self.__root_key = None
        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(
                param_rewrites[key]
            )
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(
                    key_rewrites[key]
                )
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> str:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        param_rewrites, keys_rewrites = self.resolve_rewrites(context)
        data = yaml.safe_load(open(yaml_filename))
        self.substitute_params(data, param_rewrites)
        self.add_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context: launch.LaunchContext) -> \
            tuple[dict[str, str], dict[str, str]]:
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(
                context, self.__param_rewrites[key]
            )
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(
                context, self.__key_rewrites[key]
            )
        return resolved_params, resolved_keys

    def substitute_params(self, yaml: dict[str, YamlValue],
                          param_rewrites: dict[str, str]) -> None:
        # substitute leaf-only parameters
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)

    def add_params(self, yaml: dict[str, YamlValue],
                   param_rewrites: dict[str, str]) -> None:
        # add new total path parameters
        yaml_paths = self.pathify(yaml)
        for path in param_rewrites:
            if not path in yaml_paths:  # noqa: E713
                new_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                if 'ros__parameters' in yaml_keys:
                    yaml = self.updateYamlPathVals(yaml, yaml_keys, new_val)

    def updateYamlPathVals(
            self, yaml: dict[str, YamlValue],
            yaml_key_list: list[str], rewrite_val: YamlValue) -> dict[str, YamlValue]:

        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                yaml[int(key)] = self.updateYamlPathVals(
                    yaml[int(key)], yaml_key_list, rewrite_val
                )
            else:
                yaml[key] = self.updateYamlPathVals(  # type: ignore[assignment]
                    yaml.get(key, {}),  # type: ignore[arg-type]
                    yaml_key_list,
                    rewrite_val
                )
        return yaml

    def substitute_keys(
            self, yaml: dict[str, YamlValue], key_rewrites: dict[str, str]) -> None:
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                if isinstance(val, dict):
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData: dict[str, YamlValue]) -> \
            Generator[DictItemReference, None, None]:
        if not isinstance(yamlData, dict):
            return

        for key in yamlData.keys():
            child = yamlData[key]

            if isinstance(child, dict):
                # Recursively process nested dictionaries
                yield from self.getYamlLeafKeys(child)

            yield DictItemReference(yamlData, key)

    def pathify(
            self, d: Union[dict[str, YamlValue], list[YamlValue], YamlValue],
            p: Optional[str] = None,
            paths: Optional[dict[str, YamlValue]] = None,
            joinchar: str = '.') -> dict[str, YamlValue]:
        if p is None:
            paths = {}
            self.pathify(d, '', paths, joinchar=joinchar)
            return paths

        assert paths is not None
        pn = p
        if p != '':
            pn += joinchar
        if isinstance(d, dict):
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            paths[p] = d
        return paths

    def convert(self, text_value: str) -> YamlValue:
        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if '.' in text_value else int(text_value)
            except ValueError:
                pass

        # try converting to bool
        if text_value.lower() == 'true':
            return True
        if text_value.lower() == 'false':
            return False

        # nothing else worked so fall through and return text
        return text_value
