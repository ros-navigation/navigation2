from bt_nodes_validation.validate_bt_xml_nodes import extract_code_port_data

import pytest


class TestMissingPortData:
    """Test cases for missing port data."""

    def test_missing_arguments(self):
        """Test missing arguments."""
        content = 'BT::InputPort<int>()'
        with pytest.raises(
                ValueError,
                match='Failed to extract port arguments: no arguments found.'
        ):
            extract_code_port_data(content)

    def test_port_is_not_quoted(self):
        """Test port name is not quoted."""
        content = 'BT::InputPort<int>(port, "Description")'
        with pytest.raises(
                ValueError,
                match='Port name must be a quoted string.'
        ):
            extract_code_port_data(content)

    def test_missing_port_name(self):
        """Test missing port name."""
        content = 'BT::InputPort<int>("", "Description")'
        with pytest.raises(ValueError, match='Failed to extract port name: empty string.'):
            extract_code_port_data(content)


class TestSuccessfulExtractionPortData:
    """Test successful extraction of port data."""

    # ==================== INPUT PORT, OUTPUT PORT AND BIDIRECTIONAL PORT ====================

    def test_input_port_detection(self):
        """Test InputPort detection."""
        content = 'BT::InputPort<int>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': True
            }
        }

    def test_output_port_detection(self):
        """Test OutputPort detection."""
        content = 'BT::OutputPort<int>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': True
            }
        }

    def test_bidirectional_port_detection(self):
        """Test BidirectionalPort detection."""
        content = 'BT::BidirectionalPort<int>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': True
            }
        }

    # ==================== DATA TYPES ====================

    def test_builtin_data_type(self):
        """Test built-in data type extraction."""
        content = 'BT::InputPort<double>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'double', 'default': '', 'has_description': True
            }
        }

    def test_std_builtin_data_type(self):
        """Test std built-in data type extraction."""
        content = 'BT::InputPort<std::string>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'string', 'default': '', 'has_description': True
            }
        }

    def test_custom_data_type(self):
        """Test custom data type extraction."""
        content = 'BT::InputPort<ActionResult::_planning_time_type>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'builtin_interfaces::msg::Duration',
                'default': '',
                'has_description': True
            }
        }

    # ==================== COMPLEX DATA TYPES ====================

    def test_data_type_with_surrounding_whitespace(self):
        """Test custom data type extraction with surrounding whitespace."""
        content = 'BT::InputPort< double >("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'double',
                'default': '',
                'has_description': True
            }
        }

    def test_nested_data_type(self):
        """Test nested data type extraction."""
        content = 'BT::InputPort<std::vector<std::string>>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'vector<string>',
                'default': '',
                'has_description': True
            }
        }

    # ==================== BASIC ARGUMENTS ====================

    def test_single_port_argument(self):
        """Test extracting single quoted string argument."""
        content = 'BT::InputPort<int>("port")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': False
            }
        }

    def test_two_arguments_name_and_description(self):
        """Test two arguments: name and description."""
        content = 'BT::InputPort<int>("port", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': True
            }
        }

    def test_three_arguments_with_double_default(self):
        """Test three arguments with double default value."""
        content = 'BT::InputPort<double>("port", 1.0, "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'double', 'default': '1.0', 'has_description': True
            }
        }

    def test_three_arguments_with_int_default(self):
        """Test three arguments with integer default value."""
        content = 'BT::InputPort<int>("port", 1, "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '1', 'has_description': True
            }
        }

    def test_three_arguments_with_bool_default(self):
        """Test three arguments with boolean default."""
        content = 'BT::InputPort<bool>("port", true, "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'bool', 'default': 'true', 'has_description': True
            }
        }

    def test_three_arguments_with_string_default(self):
        """Test three arguments with non-empty string default."""
        content = 'BT::InputPort<std::string>("port", "base_link", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'string', 'default': '"base_link"', 'has_description': True
            }
        }

    # ==================== COMPLEX DEFAULT VALUES ====================

    def test_function_call_as_default_value(self):
        """Test function call as default value."""
        content = """
            BT::InputPort<double>("port", std::numeric_limits<double>::infinity(), "Description")
        """
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'double',
                'default': 'numeric_limits<double>::infinity()',
                'has_description': True
            }
        }

    def test_negative_default_value(self):
        """Test negative numeric default value."""
        content = 'BT::InputPort<double>("port", -1.0, "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'double', 'default': '-1.0', 'has_description': True
            }
        }

    def test_nested_template_in_default(self):
        """Test nested templates in default value."""
        content = 'BT::InputPort<int>("port", std::vector<int>{1, 2, 3}, "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': 'vector<int>{1, 2, 3}', 'has_description': True
            }
        }

    def test_initializer_list_default(self):
        """Test initializer list as default value."""
        content = 'BT::InputPort<int>("port", std::map<int,int>{}, "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': 'map<int,int>{}', 'has_description': True
            }
        }

    def test_complex_initializer_list(self):
        """Test complex initializer list with nested structures."""
        content = """
            BT::InputPort<int>(
                "port",
                std::map<std::string, std::vector<int>>{{"key", {1, 2}}},
                "Description"
            )
        """
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int',
                'default': 'map<string, vector<int>>{{"key", {1, 2}}}',
                'has_description': True
            }
        }

    def test_empty_default_value(self):
        """Test with empty default value."""
        content = 'BT::InputPort<std::string>("port", "", "Description")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'string', 'default': '""', 'has_description': True
            }
        }

    # ==================== COMPLEX DESCRIPTIONS ====================

    def test_description_with_brackets_in_array_format(self):
        """Test description containing array notation."""
        content = 'BT::InputPort<int>("port", "[[x1,y1],[x2,y2],...] ()")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': True
            }
        }

    def test_multiline_description(self):
        """Test multiline description."""
        content = """
            BT::InputPort<int>("port", "String 1" "String 2" "String 3")
        """
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': True
            }
        }

    def test_empty_description(self):
        """Test with empty description."""
        content = 'BT::InputPort<int>("port", "")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': False
            }
        }

    def test_empty_description_with_whitespace(self):
        """Test with empty description containing only whitespace."""
        content = 'BT::InputPort<int>("port", " ")'
        result = extract_code_port_data(content)
        assert result == {
            'port': {
                'data_type': 'int', 'default': '', 'has_description': False
            }
        }
