from __future__ import annotations

import xml.etree.ElementTree as ET

from bt_nodes_validation.test.file_templates import xml_node_template, xml_template
from bt_nodes_validation.validate_bt_xml_nodes import extract_xml_nodes_data

import pytest


TREE_NODES_MODEL_TAG = 'TreeNodesModel'


def _create_elementtree(xml_string: str) -> ET.ElementTree[ET.Element]:
    """Create ElementTree from XML string."""
    root = ET.fromstring(xml_string)
    return ET.ElementTree(root)


class TestMissingXMLNodeData:
    """Tests for missing XML nodes data."""

    def test_missing_root_child(self):
        """Test missing root child."""
        xml = '<root></root>'
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match=(
                    'Invalid XML structure: '
                    f'Expected <{TREE_NODES_MODEL_TAG}> element as the first child of the root.'
                )
        ):
            extract_xml_nodes_data(tree)

    def test_missing_tree_nodes_model_tag(self):
        """Test missing TreeNodesModel tag."""
        xml = """
            <root>
                <NotTreeNodesModel></NotTreeNodesModel>
            </root>
        """
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match=(
                    'Invalid XML structure: '
                    f'Expected <{TREE_NODES_MODEL_TAG}> element as the first child of the root.'
                )
        ):
            extract_xml_nodes_data(tree)

    def test_missing_node_id(self):
        """Test missing node ID."""
        xml = """
            <root>
                <TreeNodesModel>
                    <Action>
                    </Action>
                </TreeNodesModel>
            </root>
        """
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match='Each BT node must have an "ID" attribute.'
        ):
            extract_xml_nodes_data(tree)

    def test_duplicate_node_id(self):
        """Test duplicate node ID."""
        xml = """
            <root>
                <TreeNodesModel>
                    <Action ID="TestNodeID">
                    </Action>
                    <Action ID="TestNodeID">
                    </Action>
                </TreeNodesModel>
            </root>
        """
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match='Duplicate node ID found in XML: TestNodeID'
        ):
            extract_xml_nodes_data(tree)

    def test_missing_name_attribute(self):
        """Test missing "name" attribute."""
        xml = """
            <root>
                <TreeNodesModel>
                    <Action ID="TestNodeID">
                        <input_port type="bool"></input_port>
                    </Action>
                </TreeNodesModel>
            </root>
        """
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match='Each port in TestNodeID node must have a "name" attribute.'
        ):
            extract_xml_nodes_data(tree)

    def test_missing_type_attribute(self):
        """Test missing "type" attribute."""
        xml = """
            <root>
                <TreeNodesModel>
                    <Action ID="TestNodeID">
                        <input_port name="test"></input_port>
                    </Action>
                </TreeNodesModel>
            </root>
        """
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match='test port in TestNodeID node is missing a "type" attribute.'
        ):
            extract_xml_nodes_data(tree)

    def test_duplicate_port_definition(self):
        """Test duplicate port definition."""
        xml = """
            <root>
                <TreeNodesModel>
                    <Action ID="TestNodeID">
                        <input_port name="test" type="bool"></input_port>
                        <input_port name="test" type="bool"></input_port>
                    </Action>
                </TreeNodesModel>
            </root>
        """
        tree = _create_elementtree(xml)
        with pytest.raises(
                ValueError,
                match='Duplicate port name test found in TestNodeID node.'
        ):
            extract_xml_nodes_data(tree)

    def test_whitespace_only_description(self):
        """Treat whitespace-only description as empty."""
        xml_content = xml_template([
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports='<input_port name="test" type="bool"> </input_port>'
            )
        ])
        tree = _create_elementtree(xml_content)
        result = extract_xml_nodes_data(tree)
        assert result['TestNode']['test']['has_description'] is False

    def test_newline_only_description(self):
        """Treat newline-only description as empty."""
        xml_content = xml_template([
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="test" type="bool">
                    </input_port>
                """
            )
        ])
        tree = _create_elementtree(xml_content)
        result = extract_xml_nodes_data(tree)
        assert result['TestNode']['test']['has_description'] is False


class TestSuccessfulExtractXMLNodeData:
    """Tests for successful XML node data extraction."""

    def test_single_node_data_extraction(self):
        """Test single node data extraction."""
        xml_content = xml_template([
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports='<input_port name="port" type="bool">Description.</input_port>'
            )
        ])
        tree = _create_elementtree(xml_content)
        result = extract_xml_nodes_data(tree)
        assert result == {
            'TestNode': {
                'port': {
                    'data_type': 'bool',
                    'default': '',
                    'has_description': True
                }
            }
        }

    def test_port_with_default(self):
        """Test single node data extraction with default value in port."""
        xml_content = xml_template([
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="port" default="true" type="bool">
                        Description.
                    </input_port>
                """
            )
        ])
        tree = _create_elementtree(xml_content)
        result = extract_xml_nodes_data(tree)
        assert result == {
            'TestNode': {
                'port': {
                    'data_type': 'bool',
                    'default': 'true',
                    'has_description': True
                }
            }
        }

    def test_multiple_node_data_extraction(self):
        """Test multiple node data extraction."""
        xml_content = xml_template([
            xml_node_template(
                node_type='Action',
                node_id='TestNode1',
                ports='<input_port name="port" type="bool">Description.</input_port>'
            ),
            xml_node_template(
                node_type='Action',
                node_id='TestNode2',
                ports='<input_port name="port" type="double">Description.</input_port>'
            ),
            xml_node_template(
                node_type='Action',
                node_id='TestNode3',
                ports='<input_port name="port" default="0" type="int">Description.</input_port>'
            )
        ])
        tree = _create_elementtree(xml_content)
        result = extract_xml_nodes_data(tree)
        assert result == {
            'TestNode1': {
                'port': {
                    'data_type': 'bool',
                    'default': '',
                    'has_description': True
                }
            },
            'TestNode2': {
                'port': {
                    'data_type': 'double',
                    'default': '',
                    'has_description': True
                }
            },
            'TestNode3': {
                'port': {
                    'data_type': 'int',
                    'default': '0',
                    'has_description': True
                }
            }
        }

    def test_valid_description_with_surrounding_whitespace(self):
        """Test valid description with surrounding whitespace."""
        xml_content = xml_template([
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports='<input_port name="test" type="bool">  Description.  </input_port>'
            )
        ])
        tree = _create_elementtree(xml_content)
        result = extract_xml_nodes_data(tree)
        assert result['TestNode']['test']['has_description'] is True
