from bt_nodes_validation.test.file_templates import (cpp_custom_base_template,
                                                     cpp_multi_class_template, cpp_template,
                                                     hpp_base_class_template,
                                                     hpp_custom_base_template,
                                                     hpp_multi_class_template, hpp_template,
                                                     xml_node_template)


class TestNodeMismatches:
    """Test cases for node mismatches between code and XML."""

    def test_node_in_code_missing_in_xml(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when a node is registered in code but missing in XML."""
        cpp_content = [
            cpp_template(
                class_name='CodeOnlyNode',
                node_id='CodeOnlyNode'
            ),
            cpp_template(
                class_name='CommonNode',
                node_id='CommonNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='CodeOnlyNode',
                ports='BT::InputPort<int>("port", "Description")'
            ),
            hpp_template(
                class_name='CommonNode',
                ports='BT::InputPort<bool>("port", "Description")'
            )
        ]

        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='CommonNode',
                ports='<input_port name="port" type="bool">Description.</input_port>'
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'CodeOnlyNode' in result.stdout
        assert 'missing in XML' in result.stdout

    def test_node_in_xml_missing_in_code(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when a node exists in XML but is missing in code."""
        cpp_content = [
            cpp_template(
                class_name='CommonNode',
                node_id='CommonNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='CommonNode',
                ports='BT::InputPort<bool>("port", "Description")'
            )
        ]

        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='XMLOnlyNode',
                ports='<input_port name="port" type="int">Description.</input_port>'
            ),
            xml_node_template(
                node_type='Action',
                node_id='CommonNode',
                ports='<input_port name="port" type="bool">Description.</input_port>'
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'XMLOnlyNode' in result.stdout
        assert 'missing in code' in result.stdout


class TestPortMismatches:
    """Test cases for port mismatches between code and XML."""

    def test_port_in_code_missing_in_xml(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when a port exists in HPP but is missing in XML."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="input_port" type="string">Input description.</input_port>
                    <output_port name="output_port" type="int">Description.</output_port>
                """
            )
        ]

        cpp_content = [
            cpp_template(
                class_name='TestNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode',
                ports="""
                    BT::InputPort<std::string>("missing_in_xml", "Missing port in XML"),
                    BT::InputPort<std::string>("input_port", "Input description"),
                    BT::OutputPort<int>("output_port", "Description")
                """
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'missing_in_xml' in result.stdout
        assert 'missing in XML' in result.stdout

    def test_port_in_xml_missing_in_code(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when a port exists in XML but is missing in HPP."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="missing_in_code" type="string">
                        Missing port in code.
                    </input_port>
                    <input_port name="input_port" type="string">
                        Description.
                    </input_port>
                """
            )
        ]

        cpp_content = [
            cpp_template(
                class_name='TestNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode',
                ports='BT::InputPort<std::string>("input_port", "Description")'
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'missing_in_code' in result.stdout
        assert 'missing in code' in result.stdout

    def test_port_in_base_class_missing_in_xml(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when a port exists in the base class but is missing in XML."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="input_port" type="double">
                        Description.
                    </input_port>
                    <output_port name="output_port" type="double">
                        Description.
                    </output_port>
                    <output_port name="output_base_port" type="int">
                        Description.
                    </output_port>
                """
            )
        ]

        cpp_content = [
            cpp_custom_base_template(
                class_name='TestNode',
                base_class_name='TestBaseNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_custom_base_template(
                class_name='TestNode',
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<double>("input_port", "Description"),
                    BT::OutputPort<double>("output_port", "Description")
                """
            )
        ]

        base_classes = {
            'TestBaseNode': hpp_base_class_template(
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<int>("missing_in_xml", "Description"),
                    BT::OutputPort<int>("output_base_port", "Description")
                """
            )
        }

        config_path = file_creator(
            xml_content,
            cpp_content,
            hpp_content,
            base_classes
        )
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'missing_in_xml' in result.stdout
        assert 'missing in XML' in result.stdout

    def test_port_in_xml_missing_in_base_class(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when a port exists in XML but is missing in base class."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="input_port" type="double">
                        Description.
                    </input_port>
                    <input_port name="missing_in_base_class" type="int">
                        Description.
                    </input_port>
                    <output_port name="output_port" type="double">
                        Description.
                    </output_port>
                    <output_port name="output_base_port" type="int">
                        Description.
                    </output_port>
                """
            )
        ]

        cpp_content = [
            cpp_custom_base_template(
                class_name='TestNode',
                base_class_name='TestBaseNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_custom_base_template(
                class_name='TestNode',
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<double>("input_port", "Description"),
                    BT::OutputPort<double>("output_port", "Description")
                """
            )
        ]

        base_classes = {
            'TestBaseNode': hpp_base_class_template(
                base_class_name='TestBaseNode',
                ports="""
                    BT::OutputPort<int>("output_base_port", "Description")
                """
            )
        }

        config_path = file_creator(
            xml_content,
            cpp_content,
            hpp_content,
            base_classes
        )
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'missing_in_base_class' in result.stdout
        assert 'missing in code' in result.stdout

    def test_port_type_mismatch(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when port data type differs between code and XML."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="port_with_wrong_type" type="double">
                        Double type in XML.
                    </input_port>
                    <output_port name="output_port" type="string">
                        Description.
                    </output_port>
                """
            )
        ]

        cpp_content = [
            cpp_template(
                class_name='TestNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode',
                ports="""
                    BT::InputPort<int>("port_with_wrong_type", "Int type in code"),
                    BT::OutputPort<std::string>("output_port", "Description")
                """
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'data type mismatch' in result.stdout
        assert 'port_with_wrong_type' in result.stdout

    def test_port_default_value_mismatch(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when port default value differs between code and XML."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="port_with_wrong_default" default="1" type="int">
                        Default value equals 1.
                    </input_port>
                    <output_port name="output_port" type="int">
                        Description.
                    </output_port>
                """
            )
        ]

        cpp_content = [
            cpp_template(
                class_name='TestNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode',
                ports="""
                    BT::InputPort<int>("port_with_wrong_default", 0, "Default value equals 0"),
                    BT::OutputPort<int>("output_port", "Description")
                """
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'default value mismatch' in result.stdout
        assert 'port_with_wrong_default' in result.stdout


class TestDescriptionValidation:
    """Test cases for description validation."""

    def test_missing_port_description_in_xml(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection when XML port is missing description."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="port_without_description" type="bool"></input_port>
                    <output_port name="output_port" type="int">Description.</output_port>
                """
            )
        ]

        cpp_content = [
            cpp_template(
                class_name='TestNode',
                node_id='TestNode'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode',
                ports="""
                    BT::InputPort<bool>("port_without_description", "Description in code only"),
                    BT::OutputPort<int>("output_port", "Description")
                """
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert '[ERROR]' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'port_without_description' in result.stdout
        assert 'missing description' in result.stdout


class TestSuccessfulValidation:
    """Test successful validation scenarios."""

    def test_successful_validation_with_matching_code_and_xml(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test that validation succeeds when everything matches correctly."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode1',
                ports="""
                    <input_port name="input_port" default="&quot;default&quot;" type="string">
                        Description.
                    </input_port>
                    <input_port name="enable" type="bool">
                        Description.
                    </input_port>
                    <output_port name="output_port" default="0" type="int">
                        Description.
                    </output_port>
                """
            ),
            xml_node_template(
                node_type='Action',
                node_id='TestNode2',
                ports="""
                    <input_port name="distance" default="1.5" type="double">
                        Description.
                    </input_port>
                    <bidirectional_port name="output_port" type="int">
                        Description.
                    </bidirectional_port>
                """
            )
        ]

        cpp_content = [
            cpp_template(
                class_name='TestNode1',
                node_id='TestNode1'
            ),
            cpp_template(
                class_name='TestNode2',
                node_id='TestNode2'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode1',
                ports="""
                    BT::InputPort<std::string>("input_port", "default", "Description"),
                    BT::InputPort<bool>("enable", "Description"),
                    BT::OutputPort<int>("output_port", 0, "Description")
                """
            ),
            hpp_template(
                class_name='TestNode2',
                ports="""
                    BT::InputPort<double>("distance", 1.5, "Description"),
                    BT::BidirectionalPort<int>("output_port", "Description")
                """
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 0
        assert 'Validation successful' in result.stdout
        assert 'No mismatches found' in result.stdout
        assert '[ERROR]' not in result.stdout

    def test_successful_validation_with_ports_from_base_class(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test that validation succeeds when ports are extracted from the base class."""
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="input_port" type="double">
                        Description.
                    </input_port>
                    <input_port name="input_base_port" type="int">
                        Description.
                    </input_port>
                    <output_port name="output_port" type="double">
                        Description.
                    </output_port>
                    <output_port name="output_base_port" type="int">
                        Description.
                    </output_port>
                """
            ),
        ]

        cpp_content = [
            cpp_custom_base_template(
                class_name='TestNode',
                base_class_name='TestBaseNode',
                node_id='TestNode'
            ),
        ]

        hpp_content = [
            hpp_custom_base_template(
                class_name='TestNode',
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<double>("input_port", "Description"),
                    BT::OutputPort<double>("output_port", "Description")
                """
            ),
        ]

        base_classes = {
            'TestBaseNode': hpp_base_class_template(
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<int>("input_base_port", "Description"),
                    BT::OutputPort<int>("output_base_port", "Description")
                """
            )
        }

        config_path = file_creator(
            xml_content,
            cpp_content,
            hpp_content,
            base_classes
        )
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 0
        assert 'Validation successful' in result.stdout
        assert 'No mismatches found' in result.stdout
        assert '[ERROR]' not in result.stdout

    def test_successful_validation_with_multiple_classes_in_one_file(
            self,
            file_creator,
            run_validation,
            print_output):
        """
        Test that validation succeeds for multiple classes.

        When ports are extracted from different classes
        that have custom base classes, all classes are defined in one file.
        """
        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode1',
                ports="""
                    <input_port name="node1_port1" type="double">
                        Description.
                    </input_port>
                    <input_port name="input_base_port" type="int">
                        Description.
                    </input_port>
                    <output_port name="node1_port2" type="double">
                        Description.
                    </output_port>
                    <output_port name="output_base_port" type="string">
                        Description.
                    </output_port>
                """
            ),
            xml_node_template(
                node_type='Action',
                node_id='TestNode2',
                ports="""
                    <input_port name="input_base_port" type="int">
                        Description.
                    </input_port>
                    <output_port name="output_base_port" type="string">
                        Description.
                    </output_port>
                """
            ),
            xml_node_template(
                node_type='Action',
                node_id='TestNode3',
                ports="""
                    <input_port name="node3_port1" default="false" type="bool">
                        Description.
                    </input_port>
                    <input_port name="input_base_port" type="int">
                        Description.
                    </input_port>
                    <output_port name="output_base_port" type="string">
                        Description.
                    </output_port>
                """
            ),
        ]

        cpp_content = [
            cpp_multi_class_template(classes=[
                {
                    'class_name': 'TestNode1',
                    'base_class_name': 'TestBaseNode',
                    'node_id': 'TestNode1'
                },
                {
                    'class_name': 'TestNode2',
                    'base_class_name': 'TestBaseNode',
                    'node_id': 'TestNode2'
                },
                {
                    'class_name': 'TestNode3',
                    'base_class_name': 'TestBaseNode',
                    'node_id': 'TestNode3'
                }
            ]),
        ]

        hpp_content = [
            hpp_multi_class_template(classes=[
                {
                    'class_name': 'TestNode1',
                    'base_class_name': 'TestBaseNode',
                    'ports': """
                        BT::InputPort<double>("node1_port1", "Description"),
                        BT::OutputPort<double>("node1_port2", "Description")
                    """
                },
                {
                    'class_name': 'TestNode2',
                    'base_class_name': 'TestBaseNode',
                    'ports': ''
                },
                {
                    'class_name': 'TestNode3',
                    'base_class_name': 'TestBaseNode',
                    'ports': """
                        BT::InputPort<bool>("node3_port1", false, "Description"),
                    """
                }
            ]),
        ]

        base_classes = {
            'TestBaseNode': hpp_base_class_template(
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<int>("input_base_port", "Description"),
                    BT::OutputPort<std::string>("output_base_port", "Description")
                """
            )
        }

        config_path = file_creator(
            xml_content,
            cpp_content,
            hpp_content,
            base_classes
        )
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 0
        assert 'Validation successful' in result.stdout
        assert 'No mismatches found' in result.stdout
        assert '[ERROR]' not in result.stdout
