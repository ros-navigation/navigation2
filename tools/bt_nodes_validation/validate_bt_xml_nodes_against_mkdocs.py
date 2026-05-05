import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import requests
import yaml

# Mapping between XML tag names
# and their corresponding page names in the documentation
PLUGIN_TYPES = {
    'Action': 'Action Plugins',
    'Condition': 'Condition Plugins',
    'Control': 'Control Plugins',
    'Decorator': 'Decorator Plugins'
}


def fetch_github_file(repo: str, branch: str, file_path: str) -> str:
    """Fetch raw file content using GitHub API."""
    api_url = f'https://api.github.com/repos/{repo}/contents/{file_path}'
    headers = {
        'Accept': 'application/vnd.github.v3.raw',
    }
    ref = f'refs/heads/{branch}'

    try:
        response = requests.get(
            api_url,
            headers=headers,
            params={'ref': ref},
            timeout=15
        )
        response.raise_for_status()
        print(
            f'Successfully fetched {file_path} file '
            f'from GitHub repository {repo} (branch {branch}).'
        )
        return response.text
    except requests.RequestException:
        print(
            f'Failed to fetch {file_path} file '
            f'from GitHub repository {repo} (branch {branch}).'
        )
        raise


def extract_keys(data: list[dict[str, str] | str]) -> list[str]:
    """Extract keys from a list of dictionaries."""
    keys: list[str] = []
    for item in data:
        if isinstance(item, dict):
            keys.extend(item.keys())
    return keys


def get_yaml_child_keys(
    data: dict | list,
    parent_key: str
) -> list[str]:
    """Recursively search for the parent_key and return its child keys."""
    if isinstance(data, dict):
        for key, value in data.items():
            if key == parent_key:
                return extract_keys(value)
            result = get_yaml_child_keys(value, parent_key)
            if result:
                return result
    elif isinstance(data, list):
        for item in data:
            result = get_yaml_child_keys(item, parent_key)
            if result:
                return result
    return []


def get_xml_ids(root: ET.Element, tag_name: str) -> list[str]:
    """Extract 'ID' attributes of XML elements with the specified tag name."""
    xml_ids: list[str] = []
    for node in root.findall(tag_name):
        node_id = node.get('ID')
        if node_id is not None:
            xml_ids.append(node_id)
    return xml_ids


def load_bt_nodes_model(file_path: Path) -> ET.Element:
    """Load the Behavior Tree nodes model."""
    if not file_path.exists():
        raise FileNotFoundError('File path does not exist.')

    try:
        root = ET.parse(file_path).getroot()
        if not len(root):
            raise IndexError(
                f'XML root element in {file_path} has no children. '
                f'Expected <TreeNodesModel> element.'
            )
        return root[0]
    except (ET.ParseError, IndexError):
        print(f'Failed to parse file {file_path}')
        raise


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--docs_repo',
        type=str,
        required=True,
        help='GitHub repository for the Nav2 documentation'
    )
    parser.add_argument(
        '--docs_branch',
        type=str,
        required=True,
        help='GitHub branch for the Nav2 documentation'
    )
    parser.add_argument(
        '--docs_config_file_path',
        type=str,
        required=True,
        default=Path('nav.yml'),
        help='Path to the Nav2 documentation config file'
    )
    parser.add_argument(
        '--nav2_bt_nodes_file_path',
        type=Path,
        default=Path('nav2_behavior_tree/nav2_tree_nodes.xml'),
        help='Path to the Nav2 Behavior Tree XML file'
    )
    args = parser.parse_args()

    docs_repo = args.docs_repo
    docs_branch = args.docs_branch
    docs_config_file_path = args.docs_config_file_path
    nav2_tree_nodes_file_path = args.nav2_bt_nodes_file_path

    try:
        mkdocs_config = fetch_github_file(
            docs_repo,
            docs_branch,
            docs_config_file_path
        )
    except requests.RequestException as exc:
        print(f'Failed to fetch file from GitHub: {exc}')
        sys.exit(1)

    mkdocs_config_yaml = yaml.safe_load(mkdocs_config)

    nav_key = 'nav'
    try:
        docs_nav = mkdocs_config_yaml[nav_key]
    except KeyError:
        print(
            f'"{nav_key}" key not found in the MkDocs configuration file. '
            f'File: {docs_config_file_path} from {docs_repo} (branch: {docs_branch}).'
        )
        sys.exit(1)

    bt_node_names_yaml: dict[str, list[str]] = {}
    for page_name in PLUGIN_TYPES.values():
        bt_node_names_yaml[page_name] = get_yaml_child_keys(docs_nav, page_name)

    try:
        nav2_tree_nodes = load_bt_nodes_model(nav2_tree_nodes_file_path)
    except (FileNotFoundError, ET.ParseError, IndexError) as exc:
        print(
            f'Failed to load BT node model from {nav2_tree_nodes_file_path}: {exc}'
        )
        sys.exit(1)

    bt_node_names_xml: dict[str, list[str]] = {}
    for tag_name in PLUGIN_TYPES.keys():
        bt_node_names_xml[tag_name] = get_xml_ids(nav2_tree_nodes, tag_name)

    print(f'\nComparing BT Nodes between {docs_config_file_path} '
          f'from {docs_repo} (branch {docs_branch}) '
          f'and {nav2_tree_nodes_file_path} from the Nav2 project...\n')

    mismatch_found = False
    for tag_name, page_name in PLUGIN_TYPES.items():
        yaml_names = set(bt_node_names_yaml.get(page_name, []))
        xml_names = set(bt_node_names_xml.get(tag_name, []))

        if yaml_names != xml_names:
            print(f'Mismatch in {tag_name} BT Nodes:')
            mismatch_found = True

            yaml_xml_diff = yaml_names - xml_names
            if yaml_xml_diff:
                print(
                    '\tMissing nodes in Nav2 project XML: '
                    f'{", ".join(sorted(yaml_xml_diff))}'
                )

            xml_yaml_diff = xml_names - yaml_names
            if xml_yaml_diff:
                print(
                    '\tMissing nodes in Nav2 documentation YAML: '
                    f'{", ".join(sorted(xml_yaml_diff))}'
                )
        else:
            print(f'{tag_name} BT Nodes match between YAML and XML.')

    if mismatch_found:
        sys.exit(1)


if __name__ == '__main__':
    main()
