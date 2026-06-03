# BT Nodes Validation Tests

This directory contains pytest tests for the Behavior Tree nodes validation script [validate_bt_xml_nodes.py](../validate_bt_xml_nodes.py).

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
    -r tools/bt_nodes_validation/requirements.txt \
    -r tools/bt_nodes_validation/test/requirements.txt
```

> [!NOTE]
> The pip install might display warnings about missing ROS2 dependencies like:
> `generate-parameter-library-py requires setuptools, typeguard...`
> These are system package dependencies and can be ignored since
> they're not required by the validation tests.

## How to run tests

From the `navigation2` root directory:
```bash
chmod +x tools/bt_nodes_validation/test.sh &&
./tools/bt_nodes_validation/test.sh
```

## Test Configuration

The [pytest.ini](../pytest.ini) file is configured to show verbose output by default using `-vs` arguments,
which allows you to see the script output during test execution.
