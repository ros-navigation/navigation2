# BT Nodes Validation

The [validate_bt_xml_nodes.py](validate_bt_xml_nodes.py) script validates Behavior Tree node definitions
by comparing C++ code implementations against their XML definitions in `nav2_tree_nodes.xml`.

The script ensures:
- All nodes registered in code are defined in XML
- All ports match between code and XML (name, type, default value)
- All ports have descriptions in XML

## Prerequisites

Install the required dependencies using a virtual environment in the `navigation2` directory:

1. Install pip and venv if not already installed:
```bash
sudo apt install python3-pip python3-venv
```

2. Create a virtual environment and activate it:
```bash
python3 -m venv venv &&
source venv/bin/activate
```

3. Install all required dependencies:
```bash
pip install \
    -r tools/bt_nodes_validation/requirements.txt
```

> [!NOTE]
> The pip install might display warnings about missing ROS2 dependencies like:
> `generate-parameter-library-py requires setuptools, typeguard...`
> These are system package dependencies and can be ignored since
> they're not required by the validation.

## How to use

From the `navigation2` root directory:
```bash
python3 tools/bt_nodes_validation/validate_bt_xml_nodes.py \
    --config tools/bt_nodes_validation/config.yml
```

## Configuration

The script can be configured using [config.yml](./config.yml) file.
It specifies:

- `nav2_bt_nodes_file_path`: path to the XML file containing BT node definitions
- `local_repositories`: directories containing C++ code files for BT nodes to validate
- `github_repositories` (optional): external GitHub repositories that contain BT nodes to clone and validate

## Running Tests

To run the test suite for this validation tool, see [test/README.md](./test/README.md) for detailed instructions.
