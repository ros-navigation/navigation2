#!/usr/bin/env python3

# This tool checks behavior_trees for error_code_id and error_msg
# conformity

# Example usage to check all xml files found recursively in the parent
# directory.
# python3 checkBehaviourTree.py $(find .. -name \*.xml)

import xml.etree.ElementTree as ET
import argparse
from pathlib import Path

def check_xml_attributes(xml_file):
    try:
        # Parse the XML file
        tree = ET.parse(xml_file)
        root = tree.getroot()

        # Find all elements with error_code_id attribute
        elements_with_error_code = root.findall(".//*[@error_code_id]")

        # Check if all elements with error_code_id also have error_msg
        missing_error_msg = []
        error_prefix_mismatch = []
        for element in elements_with_error_code:
            if "error_msg" not in element.attrib:
                missing_error_msg.append(element)
            else:
                error_code_id_prefix = element.attrib['error_code_id'].replace('error_code','')
                error_msg_prefix = element.attrib['error_msg'].replace('error_msg','')
                if error_code_id_prefix != error_msg_prefix:
                    error_prefix_mismatch.append(element)

        # Print results
        if missing_error_msg:
            print(f"  {xml_file} Found {len(missing_error_msg)} element(s) missing 'error_msg' attribute:")
            for element in missing_error_msg:
                print(f"    Tag: {element.tag}, Attributes: {element.attrib}")
        if error_prefix_mismatch:
            print(f"  {xml_file} Found {len(error_prefix_mismatch)} element(s) with mismatched error_code_name_prefix")
            for element in error_prefix_mismatch:
                print(f"    Tag: {element.tag}, Attributes: {element.attrib}")

    except ET.ParseError as e:
        print(f"  Error parsing XML file: {e}")
    except FileNotFoundError:
        print(f"  File not found: {xml_file}")
    except Exception as e:
        print(f"  An error occurred: {e}")

def main():
    parser = argparse.ArgumentParser(
        description="Check behaviour trees conform to error code and msg prefix requirements.")
    parser.add_argument("files", nargs="+", type=str,
                        help="BehaviorTree XML file(s) to process")
    args = parser.parse_args()

    for file_path in args.files:
        xml_path = Path(file_path)
        if xml_path.is_file() and xml_path.suffix.lower() == '.xml':
            check_xml_attributes(xml_path)
        else:
            print(f"\nSkipping {file_path}: Not a valid XML file.")

if __name__ == "__main__":
    main()

