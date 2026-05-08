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
    (re.compile(r'^std::'), ''),
]


DEFAULT_REGEX_TRANSFORMS = [
    (re.compile(r'^std::'), ''),
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
    subprocess.run([
        'git', 'clone',
        '--depth=1',
        '--filter=blob:none',
        '--sparse',
        '--branch', branch,
        github_url,
        repo_workdir,
    ], check=True, capture_output=True, text=True)

    print(f'Performing sparse checkout in {repo_workdir} directory.')
    try:
        subprocess.run([
            'git',
            'sparse-checkout',
            'set',
            '--no-cone',
            *data_to_clone,
        ], cwd=repo_workdir, check=True, capture_output=True, text=True)
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
            f'The following paths do not exist in {github_url} (branch: {branch}): '
            f'{", ".join(missing_paths)}.\n'
            'Review provided paths in tools/bt_nodes_validation/config.yml.'
        )

    print(f'Cloned the following data from {github_url} (branch: {branch}) to {repo_workdir}:')
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


def findall_in_file(pattern: re.Pattern, file_path: Path) -> list[str]:
    """Find all occurrences of a regex pattern in a file and return the matches as a list."""
    if not file_path.exists() or not file_path.is_file():
        return []
    content = file_path.read_text()
    return pattern.findall(content)


def extract_sections(
        file_path: Path,
        start_pattern: re.Pattern,
        end_pattern: re.Pattern
) -> list[str]:
    """Extract a section of text from a file based on start and end patterns."""
    if not file_path.exists() or not file_path.is_file():
        return []
    content = file_path.read_text()
    pattern = re.compile(
        rf'{start_pattern.pattern}.*?{end_pattern.pattern}',
        re.DOTALL | re.MULTILINE
    )
    result = [match.group(0) for match in pattern.finditer(content)]
    return result


def fetch_external_repos(config: dict, clone_dir: Path) -> None:
    """Fetch external repositories specified in the YAML configuration file."""
    github_repos = config.get('github_repositories', {})
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


def get_files(directories: list[Path]) -> list[Path]:
    """Get all files from the specified directories."""
    files: list[Path] = []
    for directory in directories:
        dir_path = Path(directory)
        dir_files = [file for file in dir_path.iterdir() if file.is_file()]
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
        args_start = content.find('(', start_pos)

        port_type = content[port_type_start:args_start-1].strip()
        port_type = TYPE_DIRECT_MAPPINGS.get(port_type, port_type)
        port_type = convert_with_regex(port_type, TYPE_REGEX_TRANSFORMS)

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
    Extract Behaviour Tree nodes data from the given XML file.

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
    Extract class names and their corresponding node IDs from the given list files.

    Returns dictionary mapping class names to node IDs: {class_name: node_id}
    """
    register_pattern = re.compile(
        r'register(?:NodeType|Builder)<[^>]+>\s*\(\s*"([^"]+)"'
    )
    cpp_class_pattern = re.compile(
        r'register(?:NodeType|Builder)<[^>]*::([A-Za-z_][A-Za-z0-9_]*)>'
    )

    node_cpp_data: dict[str, str] = {}
    for cpp_file in cpp_files:
        class_names = findall_in_file(cpp_class_pattern, cpp_file)
        node_ids = findall_in_file(register_pattern, cpp_file)
        node_cpp_data.update(dict(zip(class_names, node_ids)))
    return node_cpp_data


def extract_hpp_classes_and_ports_data(
    hpp_files: list[Path],
    hpp_base_classes: dict[str, Path]
) -> BTNodes:
    """
    Extract class names and their corresponding port data from the given list of header files.

    Returns dictionary mapping class names to their port data:
    {class_name: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
    """
    hpp_class_start_pattern = re.compile(
        r'^\s*class\s+([A-Za-z_][A-Za-z0-9_]*)',
        re.MULTILINE
    )
    hpp_class_end_pattern = re.compile(
        r'^\s*};',
        re.MULTILINE
    )
    hpp_base_class_pattern = re.compile(
        r'public\s+(?:[A-Za-z_][A-Za-z0-9_:]*::)?([A-Za-z_][A-Za-z0-9_]*)',
        re.MULTILINE
    )

    node_hpp_data: BTNodes = {}
    for hpp_file in hpp_files:
        # Handle multiple classes declared in one file
        class_sections = extract_sections(
            hpp_file,
            start_pattern=hpp_class_start_pattern,
            end_pattern=hpp_class_end_pattern
        )
        for class_section in class_sections:
            class_name = hpp_class_start_pattern.findall(class_section)[0]
            base_class_name = hpp_base_class_pattern.findall(class_section)[0]
            ports = extract_code_port_data(class_section)

            if base_class_name in hpp_base_classes:
                base_class_content = Path(hpp_base_classes[base_class_name]).read_text()
                base_ports = extract_code_port_data(base_class_content)
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

        cpp_dirs: list[Path] = bt.get('cpp_dir_paths', [])
        cpp_files = get_files(cpp_dirs)

        hpp_dirs: list[Path] = bt.get('hpp_dir_paths', [])
        hpp_files = get_files(hpp_dirs)

        base_classes_config: list[dict[str, Path]] = bt.get('hpp_base_classes_paths', [])
        for base_class in base_classes_config:
            hpp_base_classes.update(base_class)

        node_cpp_data = extract_cpp_classes_and_ids(cpp_files)
        node_hpp_data = extract_hpp_classes_and_ports_data(hpp_files, hpp_base_classes)

        # Combine data from cpp and hpp files by class names
        # {node_id: {port_name: {'data_type': 'x', 'default': 'y', 'has_description': bool}}}
        combined_code_data = {node_cpp_data[key]: node_hpp_data[key] for key in node_cpp_data}
        bt_node_ids_code.update(combined_code_data)

    return bt_node_ids_code


def compare_bt_nodes(bt_node_ids_code: BTNodes, bt_node_ids_xml: BTNodes) -> bool:
    """Compare BT node data extracted from code and XML."""
    mismatch_found = False

    diff_xml_code = bt_node_ids_xml.keys() - bt_node_ids_code.keys()
    if diff_xml_code:
        mismatch_found = True
        print('[ERROR] Nodes present in XML but missing in code:')
        for node in diff_xml_code:
            print(f'\t - {node}')

    diff_code_xml = bt_node_ids_code.keys() - bt_node_ids_xml.keys()
    if diff_code_xml:
        mismatch_found = True
        print('[ERROR] Nodes present in code but missing in XML:')
        for node in diff_code_xml:
            print(f'\t - {node}')

    common_nodes = bt_node_ids_code.keys() & bt_node_ids_xml.keys()
    for node_name in common_nodes:
        ports_code = bt_node_ids_code[node_name].keys()
        ports_xml = bt_node_ids_xml[node_name].keys()

        diff_ports_xml_code = ports_xml - ports_code
        if diff_ports_xml_code:
            mismatch_found = True
            print(f'[ERROR] {node_name} node: ports present in XML but missing in code:')
            for port in diff_ports_xml_code:
                print(f'\t - {port}')

        diff_ports_code_xml = ports_code - ports_xml
        if diff_ports_code_xml:
            mismatch_found = True
            print(f'[ERROR] {node_name} node: ports present in code but missing in XML:')
            for port in diff_ports_code_xml:
                print(f'\t - {port}')

        common_ports = ports_code & ports_xml
        for port in common_ports:
            port_type_code = bt_node_ids_code[node_name][port]['data_type']
            port_type_xml = bt_node_ids_xml[node_name][port]['data_type']
            if port_type_code != port_type_xml:
                mismatch_found = True
                print(f'[ERROR] {node_name} node: data type mismatch for {port} port:')
                print(f'\t Code: {port_type_code}')
                print(f'\t XML: {port_type_xml}')

            port_default_code = bt_node_ids_code[node_name][port]['default']
            port_default_xml = bt_node_ids_xml[node_name][port]['default']
            if port_default_code != port_default_xml:
                mismatch_found = True
                print(f'[ERROR] {node_name} node: default value mismatch for {port} port:')
                print(f'\t Code: {port_default_code}')
                print(f'\t XML: {port_default_xml}')
    return mismatch_found


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
        'containing repositories information and '
        'path to the nav2_tree_nodes.xml file to compare against.'
    )
    args = parser.parse_args()
    config = args.config

    try:
        repos_config_content = config.read_text()
        repos_config_yaml = yaml.safe_load(repos_config_content)
    except (FileNotFoundError, yaml.YAMLError) as exc:
        print(f'Failed to load configuration file {config}: {exc}')
        sys.exit(1)

    nav2_bt_nodes = repos_config_yaml.get(
        'nav2_bt_nodes_file_path',
        'nav2_behavior_tree/nav2_tree_nodes.xml'
    )
    nav2_bt_nodes_file_path = Path(nav2_bt_nodes)

    try:
        nav2_bt_nodes_model = load_bt_nodes_model(nav2_bt_nodes_file_path)
    except (FileNotFoundError, ET.ParseError, ValueError, IndexError) as exc:
        print(f'Failed to load BT node model from {nav2_bt_nodes_file_path}: {exc}')
        sys.exit(1)

    try:
        bt_node_ids_xml = extract_xml_nodes_data(nav2_bt_nodes_model)
    except ValueError as exc:
        print(f'Failed to extract BT nodes data from XML: {exc}')
        sys.exit(1)

    try:
        # Always copy external repositories to the navigation2 root directory,
        # regardless of the script's location
        clone_dir = git_root_path(Path(__file__).parent)
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f'Failed to determine git root directory: {exc}')
        sys.exit(1)

    print('Cloning external repositories...')
    try:
        fetch_external_repos(repos_config_yaml, clone_dir)
    except (subprocess.CalledProcessError, FileNotFoundError, OSError) as exc:
        stderr = getattr(exc, 'stderr', None)
        print(f'Failed to fetch external repositories: {stderr or exc}')
        sys.exit(1)

    local_repos_yaml = repos_config_yaml.get('local_repositories', {})
    github_repos_yaml = repos_config_yaml.get('github_repositories', {})
    update_paths_for_external_repos(github_repos_yaml, clone_dir)
    repos_yaml = local_repos_yaml | github_repos_yaml

    bt_node_ids_code = extract_code_nodes_data(repos_yaml)

    print('Comparing BT nodes data extracted from code and XML files...')
    is_mismatch_found = compare_bt_nodes(bt_node_ids_code, bt_node_ids_xml)

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
