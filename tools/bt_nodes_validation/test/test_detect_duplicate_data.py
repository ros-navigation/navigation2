from bt_nodes_validation.test.file_templates import (cpp_custom_base_template, cpp_template,
                                                     hpp_base_class_template,
                                                     hpp_custom_base_template, hpp_template,
                                                     xml_node_template)


class TestDuplicateData:
    """Test cases for duplicate data across files."""

    def test_duplicated_cpp_classes_across_files(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection of duplicate cpp class names across files."""
        cpp_content = [
            cpp_template(
                class_name='CommonClassName',
                node_id='TestNodeID1'
            ),
            cpp_template(
                class_name='CommonClassName',
                node_id='TestNodeID2'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='CommonClassName',
                ports='BT::InputPort<int>("port1", "Description")'
            ),
            hpp_template(
                class_name='CommonClassName',
                ports='BT::InputPort<bool>("port2", "Description")'
            )
        ]

        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNodeID1',
                ports='<input_port name="port1" type="int">Description.</input_port>'
            ),
            xml_node_template(
                node_type='Action',
                node_id='TestNodeID2',
                ports='<input_port name="port2" type="bool">Description.</input_port>'
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert 'Duplicate class' in result.stdout
        assert 'CommonClassName' in result.stdout

    def test_duplicated_node_ids_across_files(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection of duplicate node IDs across files."""
        cpp_content = [
            cpp_template(
                class_name='TestNode1',
                node_id='CommonNodeID'
            ),
            cpp_template(
                class_name='TestNode2',
                node_id='CommonNodeID'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode1',
                ports='BT::InputPort<int>("port1", "Description")'
            ),
            hpp_template(
                class_name='TestNode2',
                ports='BT::InputPort<bool>("port2", "Description")'
            )
        ]

        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='CommonNodeID',
                ports='<input_port name="port1" type="int">Description.</input_port>'
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert 'Duplicate node ID' in result.stdout
        assert 'CommonNodeID' in result.stdout

    def test_duplicated_hpp_classes_across_files(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection of duplicate hpp class names across files."""
        cpp_content = [
            cpp_template(
                class_name='TestNode',
                node_id='TestNodeID'
            )
        ]

        hpp_content = [
            hpp_template(
                class_name='TestNode',
                ports='BT::InputPort<int>("port1", "Description")'
            ),
            hpp_template(
                class_name='CommonClassName',
                ports='BT::InputPort<int>("port2", "Description")'
            ),
            hpp_template(
                class_name='CommonClassName',
                ports='BT::InputPort<bool>("port3", "Description")'
            )
        ]

        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNodeID',
                ports='<input_port name="port1" type="int">Description.</input_port>'
            )
        ]

        config_path = file_creator(xml_content, cpp_content, hpp_content)
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert 'Duplicate class' in result.stdout
        assert 'CommonClassName' in result.stdout

    def test_duplicated_ports_from_base_class(
            self,
            file_creator,
            run_validation,
            print_output):
        """Test detection of duplicate ports from base class."""
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
                    BT::InputPort<double>("common_port", "Description"),
                    BT::OutputPort<double>("output_port", "Description")
                """
            ),
        ]

        base_classes = {
            'TestBaseNode': hpp_base_class_template(
                base_class_name='TestBaseNode',
                ports="""
                    BT::InputPort<int>("common_port", "Description"),
                    BT::OutputPort<int>("output_base_port", "Description")
                """
            )
        }

        xml_content = [
            xml_node_template(
                node_type='Action',
                node_id='TestNode',
                ports="""
                    <input_port name="common_port" type="int">
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

        config_path = file_creator(
            xml_content,
            cpp_content,
            hpp_content,
            base_classes
        )
        result = run_validation(config_path)

        print_output(result.stdout)
        assert result.returncode == 1
        assert 'Port name conflict between' in result.stdout
        assert 'TestNode' in result.stdout
        assert 'TestBaseNode' in result.stdout
        assert 'common_port' in result.stdout
