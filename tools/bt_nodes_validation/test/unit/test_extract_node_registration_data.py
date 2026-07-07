from bt_nodes_validation.validate_bt_xml_nodes import extract_node_registration_data

import pytest


class TestMissingNodeRegistrationData:
    """Test cases for missing node registration data."""

    def test_no_node_register(self):
        """Test case when no node registration pattern found."""
        content = """
            BT_REGISTER_NODES(factory)
            {
            }
        """
        with pytest.raises(ValueError, match='No node registration found.'):
            extract_node_registration_data(content)

    def test_missing_template_data(self):
        """Test case when node registration template data is missing (class name)."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<>("TestNodeID");
            }
        """
        with pytest.raises(
            ValueError,
            match='Failed to extract node registration template data.'
        ):
            extract_node_registration_data(content)

    def test_failed_class_name_extraction(self):
        """Test case when node registration class name extraction fails."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<0>("TestNodeID");
            }
        """
        with pytest.raises(
            ValueError,
            match='Failed to extract class name from node registration.'
        ):
            extract_node_registration_data(content)

    def test_duplicate_class_name(self):
        """Test case when class name is duplicated."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<TestNode1>("TestNodeID1");
                factory.registerNodeType<TestNode1>("TestNodeID2");
            }
        """
        with pytest.raises(
            ValueError,
            match='Duplicate TestNode1 class found.'
        ):
            extract_node_registration_data(content)

    def test_missing_arguments(self):
        """Test case when node registration arguments are missing (node ID)."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<TestNode>();
            }
        """
        with pytest.raises(
            ValueError,
            match='Failed to extract node ID from node registration: no arguments found.'
        ):
            extract_node_registration_data(content)

    def test_node_id_is_not_quoted(self):
        """Test case when node ID is not a quoted string."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<TestNode>(TestNodeID);
            }
        """
        with pytest.raises(
            ValueError,
            match='Node ID must be a quoted string.'
        ):
            extract_node_registration_data(content)

    def test_empty_node_id(self):
        """Test case when node ID is empty string."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<TestNode>("");
            }
        """
        with pytest.raises(
            ValueError,
            match='Failed to extract node ID from node registration: empty string.'
        ):
            extract_node_registration_data(content)

    def test_duplicate_node_id(self):
        """Test case when node ID name is duplicated."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<TestNode1>("TestNodeID");
                factory.registerNodeType<TestNode2>("TestNodeID");
            }
        """
        with pytest.raises(
            ValueError,
            match='Duplicate node ID registration for TestNode2 class found: TestNodeID'
        ):
            extract_node_registration_data(content)


class TestSuccessfulNodeRegistrationDataExtraction:
    """Test cases for node registration data extraction."""

    #  ==================== REGISTER NODE TYPE AND BUILDER ====================

    def test_register_node_type_detection(self):
        """Test detection of registerNodeType pattern."""
        content = """
        BT_REGISTER_NODES(factory)
        {
          factory.registerNodeType<TestNode>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_register_builder_detection(self):
        """Test detection of registerBuilder pattern."""
        content = """
            BT_REGISTER_NODES(factory)
            {
              BT::NodeBuilder builder =
                [](const std::string & name, const BT::NodeConfiguration & config)
                  {
                    return std::make_unique<TestNode>(name, "test", config);
                  };
              factory.registerBuilder<TestNode>("TestNodeID", builder);
            }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_multiple_registrations_in_one_file(self):
        """Test detection of multiple node registrations in the one file."""
        content = """
            BT_REGISTER_NODES(factory)
            {
                factory.registerNodeType<TestNode1>("TestNodeID1");
                factory.registerNodeType<TestNode2>("TestNodeID2");
                factory.registerNodeType<TestNode3>("TestNodeID3");
                factory.registerNodeType<TestNode4>("TestNodeID4");
            }
        """
        result = extract_node_registration_data(content)
        expected = {
            'TestNode1': 'TestNodeID1',
            'TestNode2': 'TestNodeID2',
            'TestNode3': 'TestNodeID3',
            'TestNode4': 'TestNodeID4'
        }
        assert result == expected

    #  ==================== COMPLEX CLASS NAMES ====================

    def test_class_with_namespace(self):
        """Test class extraction with namespace."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<name::TestNode>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_nested_namespace(self):
        """Test class extraction with nested namespace."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<outer::inner::TestNode>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_templated_class_single_param(self):
        """Test registration of templated class with single parameter."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<name::TestNode<int>>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_templated_class_multiple_params(self):
        """Test registration of templated class with multiple parameters."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<name::TestNode<int, std::string, bool>>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_nested_template_types(self):
        """Test registration of class with nested template parameters."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<name::TestNode<std::vector<int>>>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID'}

    def test_class_name_with_numbers(self):
        """Test class names containing numbers."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<name::TestNode2D>("TestNodeID");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode2D': 'TestNodeID'}

    # ==================== COMPLEX NODE IDS ====================

    def test_node_id_with_numbers(self):
        """Test node IDs containing numbers."""
        content = """
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<TestNode>("TestNodeID2D");
        }
        """
        result = extract_node_registration_data(content)
        assert result == {'TestNode': 'TestNodeID2D'}
