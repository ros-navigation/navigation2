from __future__ import annotations

import argparse
from pathlib import Path
import re
from shutil import rmtree
import subprocess
import sys
from typing import Any, TypedDict
import xml.etree.ElementTree as ET

import yaml

TYPE_DIRECT_MAPPINGS = {
    'uint16_t': 'uint16',
    'unsigned short': 'uint16',
    'ActionResult::_error_code_type': 'uint16',
    'Action::Result::_error_code_type': 'uint16',
    'ActionResult::_num_retries_type': 'uint16',
    'ActionResult::_planning_time_type': 'builtin_interfaces::msg::Duration',
    'ActionResult::_coverage_path_type': 'opennav_coverage_msgs::msg::PathComponents',
    'ActionResult::_success_type': 'bool',
    'ActionResult::_route_type': 'nav2_msgs::msg::Route',
    'ActionResult::_total_elapsed_time_type': 'builtin_interfaces::msg::Duration',
}


TYPE_REGEX_TRANSFORMS = [
    (re.compile(r'std::'), ''),
]


DEFAULT_REGEX_TRANSFORMS = [
    (re.compile(r'std::'), ''),
]


TREE_NODES_MODEL_TAG = 'TreeNodesModel'


class PortData(TypedDict):
    data_type: str
    default: str
    has_description: bool


type NodePorts = dict[str, PortData]  # {port_name: PortData}
type BTNodes = dict[str, NodePorts]  # {node_id: {port_name: PortData}}
type CPPData = dict[str, str]  # {class_name: node_id}
type HPPData = dict[str, NodePorts]  # {class_name: {port_name: PortData}}


def git_root_path(path: Path) -> Path:
    """Get the full path to the git root directory given any internal directory path."""
    repo_path = subprocess.check_output(
        ['git', 'rev-parse', '--show-toplevel'],
        text=True,
        cwd=path,
    ).strip()
    return Path(repo_path)


def clone_sparse_github_data(
    repo_name: str,
    owner: str,
    branch: str,
    data_to_clone: list[str],
    clone_dir: Path
) -> None:
    """Clone GitHub repository sparsely and checkout the specified directories."""
    repo_workdir = clone_dir / repo_name
    if repo_workdir.exists():
        rmtree(repo_workdir)

    github_url = f'https://github.com/{owner}/{repo_name}.git'
    print(f'Cloning data from {github_url} (branch: {branch}).')

    try:
        subprocess.run([
            'git', 'clone',
            '--depth=1',
            '--filter=blob:none',
            '--sparse',
            '--branch', branch,
            github_url,
            repo_workdir,
        ], check=True, text=True)

        print(f'Performing sparse checkout in {repo_workdir} directory...')
        subprocess.run([
            'git',
            'sparse-checkout',
            'set',
            '--no-cone',
            *data_to_clone,
        ], cwd=repo_workdir, check=True, text=True)
    except subprocess.CalledProcessError:
        rmtree(repo_workdir, ignore_errors=True)
        raise

    missing_paths = []
    for path in data_to_clone:
        full_path = repo_workdir / path
        if not full_path.exists():
            missing_paths.append(path)

    if missing_paths:
        rmtree(repo_workdir, ignore_errors=True)
        raise FileNotFoundError(
            f'Following paths do not exist in {github_url} (branch: {branch}): '
            f'{", ".join(missing_paths)}.\n'
        )

    print(f'Cloned following data from {github_url} (branch: {branch}) to {repo_workdir}:')
    for path in data_to_clone:
        print(f'\t - {path}')


def fetch_external_repos(github_repos: dict, clone_dir: Path) -> None:
    """Fetch external repositories specified in the YAML configuration file."""
    for repo_name, repo_info in github_repos.items():

        bt_paths: list[str] = []
        bt = repo_info.get('behavior_trees', {})
        for dir_paths in ['cpp_dir_paths', 'hpp_dir_paths']:
            bt_paths.extend(bt.get(dir_paths, []))

        base_classes: list[dict[str, str]] = bt.get('hpp_base_classes_paths', [])
        for base_class in base_classes:
            bt_paths.extend(base_class.values())

        clone_sparse_github_data(
            repo_name=repo_name,
            owner=repo_info['owner'],
            branch=repo_info['branch'],
            data_to_clone=bt_paths,
            clone_dir=clone_dir
        )


def update_paths_for_external_repos(config: dict, clone_dir: Path) -> None:
    """
    Update the directory paths for external repositories in-place.

    Points paths to the cloned location and adds repository name.
    """
    for repo_name, repo_info in config.items():
        bt = repo_info.get('behavior_trees', {})
        for key in ['cpp_dir_paths', 'hpp_dir_paths']:
            if key in bt:
                updated_paths = []
                for path in bt[key]:
                    updated_path = clone_dir / repo_name / path
                    updated_paths.append(updated_path)
                bt[key] = updated_paths

        base_classes: list[dict[str, str]] = bt.get('hpp_base_classes_paths', [])
        for base_class in base_classes:
            for class_name, path in base_class.items():
                updated_path = clone_dir / repo_name / path
                base_class[class_name] = updated_path


def get_files(directories: list[str], pattern: str) -> list[Path]:
    """Recursively get all files matching given pattern from the specified list of directories."""
    files: list[Path] = []
    for directory in directories:
        dir_path = Path(directory)
        dir_files = [file for file in dir_path.rglob(pattern) if file.is_file()]
        files.extend(dir_files)
    return files


def convert_with_regex(value: str, patterns: list[tuple[re.Pattern, str]]) -> str:
    """Apply regex patterns to convert the given value."""
    result = value
    for pattern, replacement in patterns:
        result = pattern.sub(replacement, result)
    return result


def has_leading_comments(content: str, pos: int, comment_symbol: str) -> bool:
    """Check if there are leading comments before the given position."""
    line_start = content.rfind('\n', 0, pos) + 1
    leading_content = content[line_start:pos].strip()
    return leading_content.startswith(comment_symbol)


def is_quoted_string(string: str) -> bool:
    """Check if the given string is quoted."""
    return (string.startswith('"') and string.endswith('"'))


def extract_template_data(content: str, template_start_pos: int) -> tuple[str, int]:
    """
    Extract template data starting from given position.

    Return extracted template and the position of the closing angle bracket.
    """
    angle_bracket_count = 0
    template_end_pos = -1
    pos = template_start_pos
    while pos < len(content):
        char = content[pos]
        if char == '<':
            angle_bracket_count += 1
        if char == '>':
            angle_bracket_count -= 1
            if angle_bracket_count == 0:
                template_end_pos = pos
                break
        pos += 1
    else:
        raise ValueError('Failed to extract template data: unmatched angle brackets.')
    template_data = content[template_start_pos + 1:template_end_pos].strip()
    return (template_data, template_end_pos)


def extract_arguments(content: str, args_start_pos: int) -> list[str]:
    """Extract arguments from a function starting from given position."""
    args: list[str] = []

    parentheses_count = 0
    angle_brackets_count = 0
    curly_brackets_count = 0
    inside_quote = False
    new_arg_start = args_start_pos + 1
    pos = args_start_pos
    while pos < len(content):
        char = content[pos]

        if char == '"' and (pos == 0 or content[pos-1] != '\\'):
            inside_quote = not inside_quote

        if inside_quote:
            pos += 1
            continue

        match char:
            case '(':
                parentheses_count += 1
            case ')':
                parentheses_count -= 1
                if parentheses_count == 0:
                    arg = content[new_arg_start:pos].strip()
                    if arg:
                        args.append(arg)
                    break
            case '<':
                angle_brackets_count += 1
            case '>':
                angle_brackets_count -= 1
            case '{':
                curly_brackets_count += 1
            case '}':
                curly_brackets_count -= 1

        if angle_brackets_count or curly_brackets_count or parentheses_count > 1:
            pos += 1
            continue

        if char == ',':
            arg = content[new_arg_start:pos].strip()
            if arg:
                args.append(arg)
            pos += 1
            new_arg_start = pos
            continue
        pos += 1
    else:
        raise ValueError('Failed to extract arguments: unmatched parentheses.')
    return args


def extract_code_port_data(content: str) -> NodePorts:
    """
    Extract port information from the code.

    Returns dictionary mapping port names to their data:
    {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}
    """
    ports: NodePorts = {}

    ports_code_pattern = re.compile(r'BT::(?:Input|Output|Bidirectional)Port')
    for port_match in ports_code_pattern.finditer(content):
        start_pos = port_match.end()

        if has_leading_comments(content, start_pos, '//'):
            continue

        port_type, port_type_end = extract_template_data(content, start_pos)
        port_type = TYPE_DIRECT_MAPPINGS.get(port_type, port_type)
        port_type = convert_with_regex(port_type, TYPE_REGEX_TRANSFORMS)

        args_start = content.find('(', port_type_end)
        if args_start == -1:
            raise ValueError('Failed to extract port arguments: opening parenthesis not found.')
        port_args = extract_arguments(content, args_start)
        args_number = len(port_args)
        if not args_number:
            raise ValueError('Failed to extract port arguments: no arguments found.')

        port_name = port_args[0]
        is_quoted = is_quoted_string(port_name)
        if not is_quoted:
            raise ValueError('Port name must be a quoted string.')

        port_name = port_name.strip('"')
        if not port_name:
            raise ValueError('Failed to extract port name: empty string.')

        match args_number:
            case 1:
                # Port name only, no default value or description
                ports[port_name] = {
                    'data_type': port_type,
                    'default': '',
                    'has_description': False
                }
            case 2:
                # Port name and description exist, no default value
                port_description_exists = bool((port_args[1]).strip('"').strip())
                ports[port_name] = {
                    'data_type': port_type,
                    'default': '',
                    'has_description': port_description_exists
                }
            case 3:
                # Port name, default value, and description exist
                port_default = port_args[1]
                port_default = convert_with_regex(port_default, DEFAULT_REGEX_TRANSFORMS)
                port_description_exists = bool((port_args[2]).strip('"').strip())
                ports[port_name] = {
                    'data_type': port_type,
                    'default': port_default,
                    'has_description': port_description_exists
                }
    return ports


def validate_bt_xml_structure(root: ET.Element) -> ET.Element:
    if root is None or len(root) == 0:
        raise ValueError(
            'Invalid XML structure: '
            f'Expected <{TREE_NODES_MODEL_TAG}> element as the first child of the root.'
        )
    bt_nodes_model = root[0]

    if bt_nodes_model.tag != TREE_NODES_MODEL_TAG:
        raise ValueError(
            'Invalid XML structure: '
            f'Expected <{TREE_NODES_MODEL_TAG}> element as the first child of the root.'
        )
    return bt_nodes_model


def extract_xml_nodes_data(content: ET.ElementTree[Any]) -> BTNodes:
    """
    Extract Behavior Tree nodes data from the given XML data.

    Returns dictionary mapping node IDs to their port data:
    {node_id:  {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
    """
    root = content.getroot()
    bt_nodes_model = validate_bt_xml_structure(root)

    bt_node_ids_xml: BTNodes = {}
    for node in bt_nodes_model:
        node_id = node.get('ID')
        if not node_id:
            raise ValueError('Each BT node must have an "ID" attribute.')
        if node_id in bt_node_ids_xml:
            raise ValueError(f'Duplicate node ID found in XML: {node_id}')

        ports: NodePorts = {}
        for port in node:
            port_name = port.get('name')
            if not port_name:
                raise ValueError(
                    f'Each port in {node_id} node must have a "name" attribute.'
                )
            if port_name in ports:
                raise ValueError(
                    f'Duplicate port name {port_name} found in {node_id} node.'
                )
            port_type = port.get('type')
            if not port_type:
                raise ValueError(
                    f'{port_name} port in {node_id} node is missing a "type" attribute.'
                )
            port_default = port.get('default', '')
            port_description_exists = bool((port.text or '').strip())
            ports[port_name] = {
                'data_type': port_type,
                'default': port_default,
                'has_description': port_description_exists
            }

        bt_node_ids_xml[node_id] = ports
    return bt_node_ids_xml


def extract_node_registration_data(content: str) -> dict[str, str]:
    """
    Extract node registration data from the given content.

    Returns dictionary mapping class names to node IDs: {class_name: node_id}
    """
    register_pattern = re.compile(r'register(?:NodeType|Builder)')

    register_data: dict[str, str] = {}
    for register_match in register_pattern.finditer(content):
        start_pos = register_match.end()

        if has_leading_comments(content, start_pos, '//'):
            continue

        register_type, register_type_end = extract_template_data(content, start_pos)
        if not register_type:
            raise ValueError('Failed to extract node registration template data.')

        register_type = register_type.split('<')[0].strip()
        class_name_match = re.match(r'(?:.*::)?([a-zA-Z_]\w*)', register_type)
        if not class_name_match:
            raise ValueError('Failed to extract class name from node registration.')
        class_name = class_name_match.group(1)

        if class_name in register_data:
            raise ValueError(f'Duplicate {class_name} class found.')

        args_start = content.find('(', register_type_end)
        if args_start == -1:
            raise ValueError(
                'Failed to extract node registration arguments: opening parenthesis not found.'
            )

        register_args = extract_arguments(content, args_start)
        if not register_args:
            raise ValueError(
                'Failed to extract node ID from node registration: no arguments found.'
            )

        node_id = register_args[0]
        is_quoted = is_quoted_string(node_id)
        if not is_quoted:
            raise ValueError('Node ID must be a quoted string.')

        node_id = node_id.strip('"')
        if not node_id:
            raise ValueError('Failed to extract node ID from node registration: empty string.')

        if node_id in register_data.values():
            raise ValueError(
                f'Duplicate node ID registration for {class_name} class found: {node_id}'
            )

        register_data[class_name] = node_id

    if not register_data:
        raise ValueError('No node registration found.')
    return register_data


def extract_cpp_classes_and_ids(cpp_files: list[Path]) -> CPPData:
    """
    Extract class names and their corresponding node IDs from the given list of source files.

    Returns dictionary mapping class names to node IDs: {class_name: node_id}
    """
    node_cpp_data: CPPData = {}
    for cpp_file in cpp_files:
        cpp_content = cpp_file.read_text()
        try:
            class_names_and_ids = extract_node_registration_data(cpp_content)
        except ValueError as exc:
            raise ValueError(
                f'Failed to extract node registration data from {cpp_file}: {exc}\n')

        all_classes = node_cpp_data.keys()
        file_classes = class_names_and_ids.keys()
        common_classes = all_classes & file_classes
        if common_classes:
            raise ValueError(
                f'Duplicate class name found in {cpp_file}: '
                f'{", ".join(common_classes)}.'
            )
        all_ids = set(node_cpp_data.values())
        file_ids = set(class_names_and_ids.values())
        common_ids = all_ids & file_ids
        if common_ids:
            raise ValueError(
                f'Duplicate node ID found in {cpp_file}: '
                f'{", ".join(common_ids)}.'
            )
        node_cpp_data.update(class_names_and_ids)
    return node_cpp_data


def extract_class_definitions(
    content: str,
) -> list[tuple[str, str, str]]:
    """
    Extract class data from headers.

    Returns a list of tuples: (class_name, base_class_name, class_section).
    """
    class_pattern = re.compile(
        r'^\s*class\s+([A-Za-z_][A-Za-z0-9_]*)',
        re.MULTILINE
    )
    base_class_pattern = re.compile(
        r'public\s+(?:[A-Za-z_][A-Za-z0-9_:]*::)?([A-Za-z_][A-Za-z0-9_]*)',
        re.MULTILINE
    )
    class_definitions: list[tuple[str, str, str]] = []

    pos = 0
    while pos < len(content):
        class_match = class_pattern.search(content, pos)
        if not class_match:
            break
        class_name = class_match.group(1)

        class_name_end = class_match.end()
        class_brace = content.find('{', class_name_end)
        if class_brace == -1:
            raise ValueError(
                'Failed to extract class definition: opening brace not found.'
            )
        base_class_area = content[class_name_end:class_brace]
        base_class_matches = base_class_pattern.findall(base_class_area)
        base_class_name = base_class_matches[0] if base_class_matches else ''

        brace_count = 1
        pos_brace_search = class_brace + 1
        while pos_brace_search < len(content):
            char = content[pos_brace_search]
            if char == '{':
                brace_count += 1
            if char == '}':
                brace_count -= 1
                if brace_count == 0:
                    pos_brace_search += 1
                    break
            pos_brace_search += 1
        else:
            raise ValueError(f'Failed to parse class section for {class_name}.')

        class_section = content[class_brace:pos_brace_search]
        class_definitions.append((class_name, base_class_name, class_section))

        pos = pos_brace_search

    return class_definitions


def extract_hpp_classes_and_ports_data(
    hpp_files: list[Path],
    hpp_base_classes: dict[str, Path]
) -> HPPData:
    """
    Extract class names and their corresponding port data from the given list of header files.

    Returns dictionary mapping class names to their port data:
    {class_name: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
    """
    node_hpp_data: HPPData = {}
    for hpp_file in hpp_files:
        content = hpp_file.read_text()
        class_definitions = extract_class_definitions(content)
        if not class_definitions:
            raise ValueError(
                f'No class definitions found in {hpp_file}.')
        for class_name, base_class_name, class_section in class_definitions:
            if class_name in node_hpp_data:
                raise ValueError(
                    f'Duplicate class name found in {hpp_file}: {class_name}.'
                )
            try:
                ports = extract_code_port_data(class_section)
            except ValueError as exc:
                raise ValueError(
                    f'Failed to extract port data for {class_name} class in {hpp_file}: {exc}'
                )
            if base_class_name in hpp_base_classes:
                base_class_path = hpp_base_classes[base_class_name]
                base_class_content = Path(base_class_path).read_text()
                try:
                    base_ports = extract_code_port_data(base_class_content)
                except ValueError as exc:
                    raise ValueError(
                        f'Failed to extract port data for {class_name} class '
                        f'in {base_class_path}: {exc}'
                    )
                base_port_names = base_ports.keys()
                node_port_names = ports.keys()
                common_port_names = base_port_names & node_port_names
                if common_port_names:
                    raise ValueError(
                        f'Port name conflict between {class_name} class and '
                        f'its base class {base_class_name}: '
                        f'{", ".join(common_port_names)}.'
                    )
                ports.update(base_ports)
            node_hpp_data[class_name] = ports
    return node_hpp_data


def extract_code_nodes_data(config: dict) -> BTNodes:
    """
    Extract BT node data from code based on the provided configuration.

    Returns dictionary mapping node IDs to their port data:
    {node_id: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
    """
    bt_node_ids_code: BTNodes = {}

    # Share base classes data between repositories
    hpp_base_classes: dict[str, Path] = {}

    for _, repo_info in config.items():
        bt = repo_info.get('behavior_trees', {})

        cpp_dirs = bt.get('cpp_dir_paths', [])
        cpp_files = get_files(cpp_dirs, '*.cpp')

        hpp_dirs = bt.get('hpp_dir_paths', [])
        hpp_files = get_files(hpp_dirs, '*.hpp')

        base_classes_config = bt.get('hpp_base_classes_paths', [])
        for base_class_data in base_classes_config:
            for base_class_name, base_class_path in base_class_data.items():
                if base_class_name in hpp_base_classes:
                    raise ValueError(
                        f'Duplicate base class name found in configuration: {base_class_name}.'
                    )
                hpp_base_classes[base_class_name] = base_class_path

        node_cpp_data = extract_cpp_classes_and_ids(cpp_files)
        node_hpp_data = extract_hpp_classes_and_ports_data(hpp_files, hpp_base_classes)

        diff_classes_cpp_hpp = node_cpp_data.keys() - node_hpp_data.keys()
        if diff_classes_cpp_hpp:
            raise ValueError(
                'Following classes are present in cpp files but missing in hpp files: '
                f'{", ".join(diff_classes_cpp_hpp)}.\n'
                'Ensure that all provided cpp files have their corresponding hpp files.'
            )
        diff_classes_hpp_cpp = node_hpp_data.keys() - node_cpp_data.keys()
        if diff_classes_hpp_cpp:
            raise ValueError(
                'Following classes are present in hpp files but missing in cpp files: '
                f'{", ".join(diff_classes_hpp_cpp)}.\n'
                'Ensure that all provided hpp files have their corresponding cpp files.'
            )

        # Combine data from cpp and hpp files by class names:
        # {node_id: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
        for class_name, node_id in node_cpp_data.items():
            bt_node_ids_code[node_id] = node_hpp_data[class_name]

    return bt_node_ids_code


def detect_bt_nodes_mismatches(bt_node_ids_code: BTNodes, bt_node_ids_xml: BTNodes) -> bool:
    """
    Compare BT node data extracted from code and XML.

    Compares node IDs, port names, data types, and default values.
    Returns True if any mismatch is found, False otherwise.
    """
    is_mismatch_found = False

    diff_node_ids_xml_code = bt_node_ids_xml.keys() - bt_node_ids_code.keys()
    if diff_node_ids_xml_code:
        is_mismatch_found = True
        print('[ERROR] Nodes present in XML but missing in code:')
        for node_id in diff_node_ids_xml_code:
            print(f'\t - {node_id}')

    diff_node_ids_code_xml = bt_node_ids_code.keys() - bt_node_ids_xml.keys()
    if diff_node_ids_code_xml:
        is_mismatch_found = True
        print('[ERROR] Nodes present in code but missing in XML:')
        for node_id in diff_node_ids_code_xml:
            print(f'\t - {node_id}')

    common_nodes = bt_node_ids_code.keys() & bt_node_ids_xml.keys()
    for node_name in common_nodes:
        ports_code = bt_node_ids_code[node_name].keys()
        ports_xml = bt_node_ids_xml[node_name].keys()

        diff_ports_xml_code = ports_xml - ports_code
        if diff_ports_xml_code:
            is_mismatch_found = True
            print(f'[ERROR] {node_name} node: ports present in XML but missing in code:')
            for port in diff_ports_xml_code:
                print(f'\t - {port}')

        diff_ports_code_xml = ports_code - ports_xml
        if diff_ports_code_xml:
            is_mismatch_found = True
            print(f'[ERROR] {node_name} node: ports present in code but missing in XML:')
            for port in diff_ports_code_xml:
                print(f'\t - {port}')

        common_ports = ports_code & ports_xml
        for port in common_ports:
            port_type_code = bt_node_ids_code[node_name][port]['data_type']
            port_type_xml = bt_node_ids_xml[node_name][port]['data_type']
            if port_type_code != port_type_xml:
                is_mismatch_found = True
                print(f'[ERROR] {node_name} node: data type mismatch for {port} port:')
                print(f'\t Code: {port_type_code}')
                print(f'\t XML: {port_type_xml}')

            port_default_code = bt_node_ids_code[node_name][port]['default']
            port_default_xml = bt_node_ids_xml[node_name][port]['default']
            if port_default_code != port_default_xml:
                is_mismatch_found = True
                print(f'[ERROR] {node_name} node: default value mismatch for {port} port:')
                print(f'\t Code: {port_default_code}')
                print(f'\t XML: {port_default_xml}')
    return is_mismatch_found


def validate_descriptions(nodes: BTNodes) -> bool:
    """Check for missing descriptions."""
    is_description_missing = False
    for node_name, ports in nodes.items():
        for port, port_data in ports.items():
            has_description = port_data['has_description']
            if not has_description:
                print(f'[ERROR] {node_name} node: missing description for {port} port.')
                is_description_missing = True
    return is_description_missing


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config',
        type=Path,
        default=Path('tools/bt_nodes_validation/config.yml'),
        help='YAML configuration file '
        'containing path to the nav2_tree_nodes.xml file '
        'and paths to repositories with code files to compare against.'
    )
    args = parser.parse_args()
    args_config = args.config

    try:
        config_yaml = args_config.read_text()
        config = yaml.safe_load(config_yaml)
    except (OSError, yaml.YAMLError) as exc:
        print(f'Failed to load configuration file {args_config}: {exc}')
        sys.exit(1)

    nav2_bt_nodes = config.get(
        'nav2_bt_nodes_file_path',
        'nav2_behavior_tree/nav2_tree_nodes.xml'
    )
    nav2_bt_nodes_file_path = Path(nav2_bt_nodes)

    try:
        bt_nodes_content = ET.parse(nav2_bt_nodes_file_path)
    except (OSError, ET.ParseError) as exc:
        print(f'Failed to load BT nodes from {nav2_bt_nodes_file_path}: {exc}')
        sys.exit(1)

    try:
        bt_node_ids_xml = extract_xml_nodes_data(bt_nodes_content)
    except (ValueError, IndexError) as exc:
        print(f'Failed to extract BT nodes data from XML: {exc}')
        sys.exit(1)

    github_repos_config = config.get('github_repositories', {})
    if github_repos_config:
        try:
            # Always copy external repositories to the navigation2 root directory,
            # regardless of the script's location
            clone_dir = git_root_path(Path(__file__).parent)
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            print(f'Failed to determine git root directory: {exc}')
            sys.exit(1)

        print('Cloning external repositories...')
        try:
            fetch_external_repos(github_repos_config, clone_dir)
        except FileNotFoundError as exc:
            print(
                f'Failed to fetch external repositories: {exc}'
                f'Review specified paths in {args_config}.'
            )
            sys.exit(1)
        except (subprocess.CalledProcessError, OSError) as exc:
            stderr = getattr(exc, 'stderr', None)
            print(f'Failed to fetch external repositories: {stderr or exc}')
            sys.exit(1)

        update_paths_for_external_repos(github_repos_config, clone_dir)

    local_repos_config = config.get('local_repositories', {})
    repos_config = local_repos_config | github_repos_config

    try:
        bt_node_ids_code = extract_code_nodes_data(repos_config)
    except (OSError, ValueError) as exc:
        print(
            f'Failed to extract BT nodes data from code: {exc}\n'
            f'Review specified files in {args_config}'
        )
        sys.exit(1)

    print('Comparing BT nodes data extracted from code and XML files...')
    is_mismatch_found = detect_bt_nodes_mismatches(bt_node_ids_code, bt_node_ids_xml)

    print('Checking for missing descriptions in XML...')
    # Skip descriptions checking in bt_node_ids_code, as they are optional.
    is_xml_description_missing = validate_descriptions(bt_node_ids_xml)

    if is_mismatch_found or is_xml_description_missing:
        print(
            'Validation failed.\n'
            'Please review BT nodes in code and '
            f'their corresponding XML definitions in {nav2_bt_nodes_file_path}.'
        )
        sys.exit(1)

    print('Validation successful. No mismatches found between code and XML BT nodes data.')


if __name__ == '__main__':
    main()
