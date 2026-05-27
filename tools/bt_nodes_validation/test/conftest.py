from pathlib import Path
import subprocess

from bt_nodes_validation.test.file_templates import config_template, xml_template

import pytest


@pytest.fixture
def file_creator(tmp_path):
    """Create test files with custom content."""
    def create_files(
        xml_nodes: list[str],
        cpp_content: list[str],
        hpp_content: list[str],
        base_classes: dict[str, str] = {}
    ) -> Path:
        test_workspace = tmp_path / 'test_workspace'
        test_workspace.mkdir()

        cpp_dir = test_workspace / 'src'
        cpp_dir.mkdir(exist_ok=True)

        hpp_dir = test_workspace / 'include'
        hpp_dir.mkdir(exist_ok=True)

        for i in range(len(cpp_content)):
            cpp_file = cpp_dir / f'test_node_{i}.cpp'
            cpp_file.write_text(cpp_content[i])

        for i in range(len(hpp_content)):
            hpp_file = hpp_dir / f'test_node_{i}.hpp'
            hpp_file.write_text(hpp_content[i])

        hpp_base_files = {}
        for base_class_name, hpp_base_content in base_classes.items():
            hpp_base_file = test_workspace / f'test_base_{base_class_name.lower()}.hpp'
            hpp_base_file.write_text(hpp_base_content)
            hpp_base_files[base_class_name] = hpp_base_file

        xml_content = xml_template(xml_nodes)

        xml_file = test_workspace / 'test_bt_nodes.xml'
        xml_file.write_text(xml_content)

        config_content = config_template(
            xml_file,
            cpp_dir,
            hpp_dir,
            hpp_base_files
        )

        config_path = test_workspace / 'config.yml'
        config_path.write_text(config_content)

        return config_path
    return create_files


@pytest.fixture
def run_validation():
    """Run the validation script with a given configuration file."""
    def validate(config_path: Path):
        script_path = Path(__file__).parent.parent / 'validate_bt_xml_nodes.py'

        if not script_path.exists():
            raise FileNotFoundError(f'Validation script not found: {script_path}')

        result = subprocess.run([
            'python3',
            script_path,
            '--config', str(config_path)],
            capture_output=True,
            text=True
        )
        return result
    return validate


@pytest.fixture
def print_output():
    """Print the captured output."""
    def print_in_frame(content: str):
        if content:
            print('\n=== VALIDATION OUTPUT ===\n')
            print(content)
            print('=========================')
    return print_in_frame
