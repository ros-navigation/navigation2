/*  Copyright (C) 2018 Davide Faconti -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "behavior_tree_core/xml_parsing.h"
#include "tinyXML2/tinyxml2.h"

#include <functional>

namespace BT
{

struct XMLParser::Pimpl
{
    TreeNode::Ptr treeParsing(const tinyxml2::XMLElement* root_element,
                              const NodeBuilder& node_builder,
                              std::vector<TreeNode::Ptr>& nodes,
                              const TreeNode::Ptr& root_parent);

    tinyxml2::XMLDocument doc;

    const BehaviorTreeFactory& factory;

    Pimpl(const BehaviorTreeFactory &fact): factory(fact) {}
};


XMLParser::XMLParser(const BehaviorTreeFactory &factory) : _p( new Pimpl(factory) )
{
}

XMLParser::~XMLParser()
{
    delete _p;
}

void XMLParser::loadFromFile(const std::string& filename)
{
    tinyxml2::XMLError err = _p->doc.LoadFile(filename.c_str());

    if (err)
    {
        char buffer[200];
        sprintf(buffer, "Error parsing the XML: %s", tinyxml2::XMLDocument::ErrorIDToName(err));
        throw std::runtime_error(buffer);
    }
}

void XMLParser::loadFromText(const std::string& xml_text)
{
    tinyxml2::XMLError err = _p->doc.Parse(xml_text.c_str(), xml_text.size());

    if (err)
    {
        char buffer[200];
        sprintf(buffer, "Error parsing the XML: %s", tinyxml2::XMLDocument::ErrorIDToName(err));
        throw std::runtime_error(buffer);
    }
}

bool XMLParser::verifyXML(std::vector<std::string>& error_messages) const
{
    error_messages.clear();

    if (_p->doc.Error())
    {
        error_messages.emplace_back("The XML was not correctly loaded");
        return false;
    }
    bool is_valid = true;

    //-------- Helper functions (lambdas) -----------------
    auto strEqual = [](const char* str1, const char* str2) -> bool {
        return strcmp(str1, str2) == 0;
    };

    auto AppendError = [&](int line_num, const std::string& text) {
        char buffer[256];
        sprintf(buffer, "Error at line %d: -> %s", line_num, text.c_str());
        error_messages.emplace_back(buffer);
        is_valid = false;
    };

    auto ChildrenCount = [](const tinyxml2::XMLElement* parent_node) {
        int count = 0;
        for (auto node = parent_node->FirstChildElement(); node != nullptr;
             node = node->NextSiblingElement())
        {
            count++;
        }
        return count;
    };

    //-----------------------------

    const tinyxml2::XMLElement* xml_root = _p->doc.RootElement();

    if (!xml_root || !strEqual(xml_root->Name(), "root"))
    {
        error_messages.emplace_back("The XML must have a root node called <root>");
        return false;
    }
    //-------------------------------------------------
    auto meta_root = xml_root->FirstChildElement("TreeNodesModel");
    auto meta_sibling = meta_root ? meta_root->NextSiblingElement("TreeNodesModel") : nullptr;

    if (meta_sibling)
    {
        AppendError(meta_sibling->GetLineNum(), " Only a single node <TreeNodesModel> is "
                                                "supported");
    }
    if (meta_root)
    {
        // not having a MetaModel is not an error. But consider that the
        // Graphical editor needs it.
        for (auto node = xml_root->FirstChildElement(); node != nullptr;
             node = node->NextSiblingElement())
        {
            const char* name = node->Name();
            if (strEqual(name, "Action") || strEqual(name, "Decorator") ||
                strEqual(name, "SubTree") || strEqual(name, "Condition"))
            {
                const char* ID = node->Attribute("ID");
                if (!ID)
                {
                    AppendError(node->GetLineNum(), "Error at line %d: -> The attribute [ID] is "
                                                    "mandatory");
                }
                for (auto param_node = xml_root->FirstChildElement("Parameter");
                     param_node != nullptr;
                     param_node = param_node->NextSiblingElement("Parameter"))
                {
                    const char* label = node->Attribute("label");
                    const char* type = node->Attribute("type");
                    if (!label || !type)
                    {
                        AppendError(node->GetLineNum(),
                                    "The node <Parameter> requires the attributes [type] and "
                                    "[label]");
                    }
                }
            }
        }
    }

    //-------------------------------------------------

    // function to be called recursively
    std::function<void(const tinyxml2::XMLElement*)> recursiveStep;

    recursiveStep = [&](const tinyxml2::XMLElement* node) {
        const int children_count = ChildrenCount(node);
        const char* name = node->Name();
        if (strEqual(name, "Decorator"))
        {
            if (children_count != 1)
            {
                AppendError(node->GetLineNum(), "The node <Decorator> must have exactly 1 child");
            }
            if (!node->Attribute("ID"))
            {
                AppendError(node->GetLineNum(), "The node <Decorator> must have the attribute "
                                                "[ID]");
            }
        }
        else if (strEqual(name, "Action"))
        {
            if (children_count != 0)
            {
                AppendError(node->GetLineNum(), "The node <Action> must not have any child");
            }
            if (!node->Attribute("ID"))
            {
                AppendError(node->GetLineNum(), "The node <Action> must have the attribute [ID]");
            }
        }
        else if (strEqual(name, "Condition"))
        {
            if (children_count != 0)
            {
                AppendError(node->GetLineNum(), "The node <Condition> must not have any child");
            }
            if (!node->Attribute("ID"))
            {
                AppendError(node->GetLineNum(), "The node <Condition> must have the attribute "
                                                "[ID]");
            }
        }
        else if (strEqual(name, "Sequence") || strEqual(name, "SequenceStar") ||
                 strEqual(name, "Fallback") || strEqual(name, "FallbackStar"))
        {
            if (children_count == 0)
            {
                AppendError(node->GetLineNum(), "A Control node must have at least 1 child");
            }
        }
        else if (strEqual(name, "SubTree"))
        {
            if (children_count > 0)
            {
                AppendError(node->GetLineNum(), "The <SubTree> node must have no children");
            }
            if (!node->Attribute("ID"))
            {
                AppendError(node->GetLineNum(), "The node <SubTree> must have the attribute [ID]");
            }
        }
        else
        {
            // Last resort:  MAYBE used ID as element name?
            bool found = false;
            for (const auto& model : _p->factory.manifests())
            {
                if (model.registration_ID == name)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                AppendError(node->GetLineNum(), std::string("Node not recognized: ") + name);
            }
        }
        //recursion
        for (auto child = node->FirstChildElement(); child != nullptr;
             child = child->NextSiblingElement())
        {
            recursiveStep(child);
        }
    };

    std::vector<std::string> tree_names;
    int tree_count = 0;

    for (auto bt_root = xml_root->FirstChildElement("BehaviorTree"); bt_root != nullptr;
         bt_root = bt_root->NextSiblingElement("BehaviorTree"))
    {
        tree_count++;
        if (bt_root->Attribute("ID"))
        {
            tree_names.push_back(bt_root->Attribute("ID"));
        }
        if (ChildrenCount(bt_root) != 1)
        {
            AppendError(bt_root->GetLineNum(), "The node <BehaviorTree> must have exactly 1 child");
        }
        else
        {
            recursiveStep(bt_root->FirstChildElement());
        }
    }

    if (xml_root->Attribute("main_tree_to_execute"))
    {
        std::string main_tree = xml_root->Attribute("main_tree_to_execute");
        if (std::find(tree_names.begin(), tree_names.end(), main_tree) == tree_names.end())
        {
            error_messages.emplace_back("The tree esecified in [main_tree_to_execute] "
                                        "can't be found");
            is_valid = false;
        }
    }
    else
    {
        if (tree_count != 1)
        {
            error_messages.emplace_back(
                "If you don't specify the attribute [main_tree_to_execute], "
                "Your file must contain a single BehaviorTree");
            is_valid = false;
        }
    }
    return is_valid;
}

TreeNode::Ptr XMLParser::instantiateTree(std::vector<TreeNode::Ptr>& nodes)
{
    nodes.clear();

    std::vector<std::string> error_messages;
    this->verifyXML(error_messages);

    if (error_messages.size() > 0)
    {
        for (const std::string& str : error_messages)
        {
            std::cerr << str << std::endl;
        }
        throw std::runtime_error("verifyXML failed");
    }

    //--------------------------------------
    tinyxml2::XMLElement* xml_root = _p->doc.RootElement();

    std::string main_tree_ID;
    if (xml_root->Attribute("main_tree_to_execute"))
    {
        main_tree_ID = xml_root->Attribute("main_tree_to_execute");
    }

    std::map<std::string, tinyxml2::XMLElement*> bt_roots;

    int tree_count = 0;

    for (auto node = xml_root->FirstChildElement("BehaviorTree"); node != nullptr;
         node = node->NextSiblingElement("BehaviorTree"))
    {
        std::string tree_name = main_tree_ID;
        if (tree_count++ > 0)
        {
            tree_name += std::string("_") + std::to_string(tree_count);
        }
        if (node->Attribute("ID"))
        {
            tree_name = node->Attribute("ID");
        }
        bt_roots[tree_name] = node;
        if (main_tree_ID.empty())
        {
            main_tree_ID = tree_name;
        }
    }

    //--------------------------------------
    NodeBuilder node_builder = [&](const std::string& ID, const std::string& name,
                                   const NodeParameters& params,
                                   TreeNode::Ptr parent) -> TreeNode::Ptr {
        TreeNode::Ptr child_node = _p->factory.instantiateTreeNode(ID, name, params);
        nodes.push_back(child_node);
        if (parent)
        {
            ControlNode* control_parent = dynamic_cast<ControlNode*>(parent.get());
            if (control_parent)
            {
                control_parent->addChild(child_node.get());
            }
            DecoratorNode* decorator_parent = dynamic_cast<DecoratorNode*>(parent.get());
            if (decorator_parent)
            {
                decorator_parent->setChild(child_node.get());
            }
        }
        DecoratorSubtreeNode* subtree_node = dynamic_cast<DecoratorSubtreeNode*>(child_node.get());

        if (subtree_node)
        {
            auto subtree_elem = bt_roots[name]->FirstChildElement();
            _p->treeParsing(subtree_elem, node_builder, nodes, child_node);
        }
        return child_node;
    };
    //--------------------------------------

    auto root_element = bt_roots[main_tree_ID]->FirstChildElement();
    return _p->treeParsing(root_element, node_builder, nodes, TreeNode::Ptr());
}

TreeNode::Ptr BT::XMLParser::Pimpl::treeParsing(const tinyxml2::XMLElement* root_element,
                                                const NodeBuilder& node_builder,
                                                std::vector<TreeNode::Ptr>& nodes,
                                                const TreeNode::Ptr& root_parent)
{
    using namespace tinyxml2;

    std::function<TreeNode::Ptr(const TreeNode::Ptr&, const tinyxml2::XMLElement*)> recursiveStep;

    recursiveStep = [&](const TreeNode::Ptr& parent,
                        const tinyxml2::XMLElement* element) -> TreeNode::Ptr {
        const std::string element_name = element->Name();
        std::string node_ID;
        std::string node_alias;
        NodeParameters node_params;

        // Actions and Decorators have their own ID
        if (element_name == "Action" || element_name == "Decorator" || element_name == "Condition")
        {
            node_ID = element->Attribute("ID");
        }
        else
        {
            node_ID = element_name;
        }

        const char* attr_alias = element->Attribute("name");
        if (attr_alias)
        {
            node_alias = attr_alias;
        }
        else
        {
            node_alias = node_ID;
        }

        if (element_name == "SubTree")
        {
            node_alias = element->Attribute("ID");
        }

        for (const XMLAttribute* att = element->FirstAttribute(); att; att = att->Next())
        {
            const std::string attribute_name = att->Name();
            if (attribute_name != "ID" && attribute_name != "name")
            {
                node_params[attribute_name] = att->Value();
            }
        }

        TreeNode::Ptr node = node_builder(node_ID, node_alias, node_params, parent);
        nodes.push_back(node);

        for (auto child_element = element->FirstChildElement(); child_element;
             child_element = child_element->NextSiblingElement())
        {
            recursiveStep(node, child_element);
        }

        return node;
    };

    // start recursion
    TreeNode::Ptr root = recursiveStep(root_parent, root_element);
    return root;
}

std::string writeXML(const BehaviorTreeFactory& factory, const TreeNode* root_node,
                     bool compact_representation)
{
    using namespace tinyxml2;

    XMLDocument doc;

    XMLElement* rootXML = doc.NewElement("root");
    doc.InsertFirstChild(rootXML);

    if (root_node)
    {
        XMLElement* bt_root = doc.NewElement("BehaviorTree");
        rootXML->InsertEndChild(bt_root);

        std::function<void(const TreeNode*, XMLElement* parent)> recursiveVisitor;

        recursiveVisitor = [&recursiveVisitor, &doc, compact_representation,
                            &factory](const TreeNode* node, XMLElement* parent) -> void {
            std::string node_type = toStr(node->type());
            std::string node_ID = node->registrationName();
            std::string node_name = node->name();

            if (node->type() == NodeType::CONTROL)
            {
                node_type = node_ID;
            }
            else if (compact_representation)
            {
                for (const auto& model : factory.manifests())
                {
                    if (model.registration_ID == node_ID)
                    {
                        node_type = node_ID;
                        break;
                    }
                }
            }

            XMLElement* element = doc.NewElement(node_type.c_str());
            if (node_type != node_ID && !node_ID.empty())
            {
                element->SetAttribute("ID", node_ID.c_str());
            }
            if (node_type != node_name && !node_name.empty() && node_name != node_ID)
            {
                element->SetAttribute("name", node_name.c_str());
            }

            for (const auto& param : node->initializationParameters())
            {
                element->SetAttribute(param.first.c_str(), param.second.c_str());
            }

            parent->InsertEndChild(element);

            if (auto control = dynamic_cast<const BT::ControlNode*>(node))
            {
                for (const auto& child : control->children())
                {
                    recursiveVisitor(static_cast<const TreeNode*>(child), element);
                }
            }
            else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
            {
                recursiveVisitor(decorator->child(), element);
            }
        };

        recursiveVisitor(root_node, bt_root);
    }
    //--------------------------

    XMLElement* model_root = doc.NewElement("TreeNodesModel");
    rootXML->InsertEndChild(model_root);

    for (auto& model : factory.manifests())
    {
        if (model.type == NodeType::CONTROL)
        {
            continue;
        }
        XMLElement* element = doc.NewElement(toStr(model.type));
        element->SetAttribute("ID", model.registration_ID.c_str());

        for (auto& param : model.required_parameters)
        {
            XMLElement* param_el = doc.NewElement("Parameter");
            param_el->SetAttribute("label", param.first.c_str());
            param_el->SetAttribute("default", param.second.c_str());
            element->InsertEndChild(param_el);
        }

        model_root->InsertEndChild(element);
    }

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return std::string(printer.CStr(), printer.CStrSize() - 1);
}

Tree buildTreeFromText(const BehaviorTreeFactory& factory, const std::string& text,
                       const Blackboard::Ptr& blackboard)
{
    XMLParser parser(factory);
    parser.loadFromText(text);

    std::vector<TreeNode::Ptr> nodes;
    auto root = parser.instantiateTree(nodes);
    assignBlackboardToEntireTree(root.get(), blackboard);
    return {root.get(), nodes};
}

Tree buildTreeFromFile(const BehaviorTreeFactory& factory, const std::string& filename,
                       const Blackboard::Ptr& blackboard)
{
    XMLParser parser(factory);
    parser.loadFromFile(filename);

    std::vector<TreeNode::Ptr> nodes;
    auto root = parser.instantiateTree(nodes);
    assignBlackboardToEntireTree(root.get(), blackboard);
    return {root.get(), nodes};
}
}
