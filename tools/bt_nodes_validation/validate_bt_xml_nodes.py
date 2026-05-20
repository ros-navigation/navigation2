import argparse
from pathlib import Path
import re
from shutil import rmtree
import subprocess
import sys
from typing import TypedDict
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


class PortData(TypedDict):
    data_type: str
    default: str
    has_description: bool


NodePorts = dict[str, PortData]  # {port_name: PortData}
BTNodes = dict[str, NodePorts]  # {node_id: {port_name: PortData}}


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
    if not data_to_clone:
        print(f'No directories specified in {repo_name} for sparse checkout.')
        return

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


def load_bt_nodes_model(file_path: Path) -> ET.Element:
    """Load the Behavior Tree nodes model."""
    root = ET.parse(file_path).getroot()

    if not len(root):
        raise IndexError(
            'Invalid XML structure: '
            'Expected <TreeNodesModel> element as the first child of the root.'
        )
    bt_nodes_model = root[0]

    if bt_nodes_model.tag != 'TreeNodesModel':
        raise ValueError(
            'Invalid XML structure: '
            'Expected <TreeNodesModel> element as the first child of the root.'
        )
    return bt_nodes_model


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


def extract_template_type(content: str, template_start_pos: int) -> str:
    """Extract the closing position of a C++ template parameter."""
    angle_bracket_count = 1
    pos = template_start_pos
    while pos < len(content):
        char = content[pos]
        if char == '<':
            angle_bracket_count += 1
        if char == '>':
            angle_bracket_count -= 1
            if angle_bracket_count == 0:
                break
        pos += 1
    else:
        raise ValueError('Failed to extract template type: unmatched angle brackets.')
    return content[template_start_pos:pos].strip()


def extract_quoted_string(content: str, start_pos: int) -> str:
    """Extract the first quoted string starting from start_pos."""
    quote_start = content.find('"', start_pos)
    if quote_start == -1:
        return ''
    quote_end = content.find('"', quote_start + 1)
    if quote_end == -1:
        return ''
    return content[quote_start+1:quote_end]


def extract_code_port_data(content: str) -> NodePorts:
    """
    Extract port information from the code.

    Returns dictionary mapping port names to their data:
    {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}
    """
    ports: NodePorts = {}

    ports_code_pattern = re.compile(r'BT::(?:Input|Output|Bidirectional)Port<')
    for file_iter in ports_code_pattern.finditer(content):
        start_pos = file_iter.start()

        if has_leading_comments(content, start_pos, '//'):
            continue

        port_type_start = content.find('<', start_pos) + 1
        port_type = extract_template_type(content, port_type_start)
        port_type = TYPE_DIRECT_MAPPINGS.get(port_type, port_type)
        port_type = convert_with_regex(port_type, TYPE_REGEX_TRANSFORMS)

        port_type_end = port_type_start + len(port_type)
        args_start = content.find('(', port_type_end)
        quote_start = content.find('"', args_start + 1)
        port_name = extract_quoted_string(content, quote_start)
        if not port_name:
            continue
        quote_end = quote_start + len(port_name) + 1

        paren_count = 0
        inside_quote = False
        has_second_arg = False
        second_arg_start = -1
        end_pos = quote_end + 1
        while end_pos < len(content):

            if content[end_pos] == '"' and (end_pos == 0 or content[end_pos-1] != '\\'):
                inside_quote = not inside_quote

            if inside_quote:
                end_pos += 1
                continue

            char = content[end_pos]
            if char == ',' and not paren_count:
                if has_second_arg:
                    # Default value and description exist
                    default = content[second_arg_start:end_pos]
                    default = convert_with_regex(default.strip(), DEFAULT_REGEX_TRANSFORMS)
                    ports[port_name] = {
                        'data_type': port_type,
                        'default': default,
                        'has_description': True
                    }
                    break
                else:
                    has_second_arg = True
                    second_arg_start = end_pos + 1

            if char == ')' and not paren_count:
                # No default value, description exists
                if has_second_arg:
                    ports[port_name] = {
                        'data_type': port_type,
                        'default': '',
                        'has_description': True
                    }
                    break
                else:
                    ports[port_name] = {
                        'data_type': port_type,
                        'default': '',
                        'has_description': False
                    }
                    break

            if char == '(':
                paren_count += 1
            elif char == ')':
                paren_count -= 1
            end_pos += 1

    return ports


def extract_xml_nodes_data(root: ET.Element) -> BTNodes:
    """
    Extract Behavior Tree nodes data from the given XML file.

    Returns dictionary mapping node IDs to their port data:
    {node_id:  {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
    """
    bt_xml: BTNodes = {}
    for node in root:
        node_id = node.get('ID')
        if not node_id:
            raise ValueError('Each BT node must have an "ID" attribute.')

        port_names: NodePorts = {}
        for port in node:
            port_name = port.get('name')
            if not port_name:
                raise ValueError(
                    f'Each port in {node_id} node must have a "name" attribute.'
                )
            port_type = port.get('type')
            if not port_type:
                raise ValueError(
                    f'{port_name} port in {node_id} node is missing a "type" attribute.'
                )
            port_default = port.get('default', '')
            port_description_exists = bool(port.text)
            port_names[port_name] = {
                'data_type': port_type,
                'default': port_default,
                'has_description': port_description_exists
            }

        bt_xml[node_id] = port_names
    return bt_xml


def extract_cpp_classes_and_ids(cpp_files: list[Path]) -> dict[str, str]:
    """
    Extract class names and their corresponding node IDs from the given list of source files.

    Returns dictionary mapping class names to node IDs: {class_name: node_id}
    """
    register_pattern = re.compile(
        r'register(?:NodeType|Builder)<(?:[^>]*::)?([A-Za-z_][A-Za-z0-9_]*)>[^(]*\([^"]*"([^"]+)"',
        re.DOTALL
    )

    node_cpp_data: dict[str, str] = {}
    for cpp_file in cpp_files:
        cpp_content = cpp_file.read_text()
        class_names_and_ids = register_pattern.findall(cpp_content)
        if not class_names_and_ids:
            raise ValueError(
                f'No node registrations found in {cpp_file}.')
        node_cpp_data.update(dict(class_names_and_ids))
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
) -> BTNodes:
    """
    Extract class names and their corresponding port data from the given list of header files.

    Returns dictionary mapping class names to their port data:
    {class_name: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
    """
    node_hpp_data: BTNodes = {}
    for hpp_file in hpp_files:
        content = hpp_file.read_text()
        class_definitions = extract_class_definitions(content)
        if not class_definitions:
            raise ValueError(
                f'No class definitions found in {hpp_file}.')
        for class_name, base_class_name, class_section in class_definitions:
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
        for base_class in base_classes_config:
            hpp_base_classes.update(base_class)

        node_cpp_data = extract_cpp_classes_and_ids(cpp_files)
        node_hpp_data = extract_hpp_classes_and_ports_data(hpp_files, hpp_base_classes)

        classes_cpp_hpp = node_cpp_data.keys() - node_hpp_data.keys()
        if classes_cpp_hpp:
            raise ValueError(
                'Following classes are present in cpp files but missing in hpp files: '
                f'{", ".join(classes_cpp_hpp)}.\n'
                'Ensure that all provided cpp files have their corresponding hpp files.'
            )
        classes_hpp_cpp = node_hpp_data.keys() - node_cpp_data.keys()
        if classes_hpp_cpp:
            raise ValueError(
                'Following classes are present in hpp files but missing in cpp files: '
                f'{", ".join(classes_hpp_cpp)}.\n'
                'Ensure that all provided hpp files have their corresponding cpp files.'
            )

        # Combine data from cpp and hpp files by class names
        # {node_id: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
        combined_code_data = {node_cpp_data[key]: node_hpp_data[key] for key in node_cpp_data}
        bt_node_ids_code.update(combined_code_data)

    return bt_node_ids_code


def detect_bt_nodes_mismatches(bt_node_ids_code: BTNodes, bt_node_ids_xml: BTNodes) -> bool:
    """
    Compare BT node data extracted from code and XML.

    Compares node IDs, port names, data types, and default values.
    Returns True if any mismatch is found, False otherwise.
    """
    is_mismatch_found = False

    diff_xml_code = bt_node_ids_xml.keys() - bt_node_ids_code.keys()
    if diff_xml_code:
        is_mismatch_found = True
        print('[ERROR] Nodes present in XML but missing in code:')
        for node in diff_xml_code:
            print(f'\t - {node}')

    diff_code_xml = bt_node_ids_code.keys() - bt_node_ids_xml.keys()
    if diff_code_xml:
        is_mismatch_found = True
        print('[ERROR] Nodes present in code but missing in XML:')
        for node in diff_code_xml:
            print(f'\t - {node}')

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
        nav2_bt_nodes_model = load_bt_nodes_model(nav2_bt_nodes_file_path)
    except (OSError, ET.ParseError, ValueError, IndexError) as exc:
        print(f'Failed to load BT node model from {nav2_bt_nodes_file_path}: {exc}')
        sys.exit(1)

    try:
        bt_node_ids_xml = extract_xml_nodes_data(nav2_bt_nodes_model)
    except ValueError as exc:
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
