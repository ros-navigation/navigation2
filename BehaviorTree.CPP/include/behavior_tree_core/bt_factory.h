/* Copyright (C) 2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018 Davide Faconti -  All Rights Reserved
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

#ifndef BT_FACTORY_H
#define BT_FACTORY_H

#include <functional>
#include <memory>
#include <map>
#include <set>
#include <cstring>
#include <algorithm>

#include "behavior_tree_core/behavior_tree.h"

namespace BT
{
/// The term "Builder" refers to the Builder Pattern (https://en.wikipedia.org/wiki/Builder_pattern)
typedef std::function<std::unique_ptr<TreeNode>(const std::string&, const NodeParameters&)>
    NodeBuilder;

/// This information is used mostly by the XMLParser.
struct TreeNodeManifest
{
    NodeType type;
    std::string registration_ID;
    NodeParameters required_parameters;
};

const char PLUGIN_SYMBOL[] = "BT_RegisterNodesFromPlugin";
#define BT_REGISTER_NODES(factory)                                                                 \
    extern "C" void __attribute__((visibility("default")))                                         \
        BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)

class BehaviorTreeFactory
{
  public:
    BehaviorTreeFactory();

    bool unregisterBuilder(const std::string& ID);

    /** More generic way to register your own builder.
     *  Most of the time you should use registerSimple???? or registerNodeType<> instead.
     */
    void registerBuilder(const std::string& ID, NodeBuilder builder);

    /// Register a SimpleActionNode
    void registerSimpleAction(const std::string& ID,
                              const SimpleActionNode::TickFunctor& tick_functor);

    /// Register a SimpleConditionNode
    void registerSimpleCondition(const std::string& ID,
                                 const SimpleConditionNode::TickFunctor& tick_functor);

    /// Register a SimpleDecoratorNode
    void registerSimpleDecorator(const std::string& ID,
                                 const SimpleDecoratorNode::TickFunctor& tick_functor);

    /**
     * @brief registerFromPlugin load a shared library and execute the function BT_REGISTER_NODES (see macro).
     *
     * This method may throw.
     *
     * @param file_path path of the file
     */
    void registerFromPlugin(const std::string file_path);

    /**
     * @brief instantiateTreeNode creates a TreeNode
     *
     * @param ID       unique ID used to register the node type
     * @param name     name of this particular instance
     * @param params   parameters (usually read from the XML definition)
     * @return         new node.
     */
    std::unique_ptr<TreeNode> instantiateTreeNode(const std::string& ID, const std::string& name,
                                                  const NodeParameters& params) const;

    /** registerNodeType is the method to use to register your custom TreeNode.
     *
     *  It accepts only classed derived from either ActionNodeBase, DecoratorNode,
     *  ControlNode or ConditionNode.
     *
     *  REMINDER: If you want your derived class to
     */
    template <typename T>
    void registerNodeType(const std::string& ID)
    {
        static_assert(std::is_base_of<ActionNodeBase, T>::value ||
                          std::is_base_of<ControlNode, T>::value ||
                          std::is_base_of<DecoratorNode, T>::value ||
                          std::is_base_of<ConditionNode, T>::value,
                      "[registerBuilder]: accepts only classed derived from either ActionNodeBase, "
                      "DecoratorNode, ControlNode or ConditionNode");

        static_assert(!std::is_abstract<T>::value,
                      "[registerBuilder]: Some methods are pure virtual. "
                      "Did you override the methods tick() and halt()?");

        constexpr bool default_constructable = std::is_constructible<T, const std::string&>::value;
        constexpr bool param_constructable =
            std::is_constructible<T, const std::string&, const NodeParameters&>::value;
        constexpr bool has_static_required_parameters =
            has_static_method_requiredNodeParameters<T>::value;

        static_assert(default_constructable || param_constructable,
                      "[registerBuilder]: the registered class must have at least one of these two "
                      "constructors: "
                      "  (const std::string&, const NodeParameters&) or (const std::string&).");

        static_assert(!(param_constructable && !has_static_required_parameters),
                      "[registerBuilder]: you MUST implement the static method: "
                      "  const NodeParameters& requiredNodeParameters();\n");

        static_assert(!(has_static_required_parameters && !param_constructable),
                      "[registerBuilder]: since you have a static method requiredNodeParameters(), "
                      "you MUST add a constructor sign signature (const std::string&, const "
                      "NodeParameters&)\n");

        registerNodeTypeImpl<T>(ID);
        storeNodeManifest<T>(ID);
    }

    /// All the builders. Made available mostly for debug purposes.
    const std::map<std::string, NodeBuilder>& builders() const;

    /// Manifests of all the registered TreeNodes.
    const std::vector<TreeNodeManifest>& manifests() const;

  private:
    std::map<std::string, NodeBuilder> builders_;
    std::vector<TreeNodeManifest> manifests_;

    // template specialization = SFINAE + black magic

    // clang-format off
    template <typename T>
    using has_default_constructor = typename std::is_constructible<T, const std::string&>;

    template <typename T>
    using has_params_constructor  = typename std::is_constructible<T, const std::string&, const NodeParameters&>;

    template <typename T, typename = void>
    struct has_static_method_requiredNodeParameters: std::false_type {};

    template <typename T>
    struct has_static_method_requiredNodeParameters<T,
            typename std::enable_if<std::is_same<decltype(T::requiredNodeParameters()), const NodeParameters&>::value>::type>
        : std::true_type {};

    template <typename T>
    typename std::enable_if< has_default_constructor<T>::value && !has_params_constructor<T>::value>::type
    registerNodeTypeImpl(const std::string& ID)
    {
        NodeBuilder builder = [](const std::string& name, const NodeParameters&)
        {
            return std::unique_ptr<TreeNode>(new T(name));
        };
        registerBuilder(ID, builder);
    }

    template <typename T>
    typename std::enable_if< !has_default_constructor<T>::value && has_params_constructor<T>::value>::type
    registerNodeTypeImpl(const std::string& ID)
    {
        NodeBuilder builder = [](const std::string& name, const NodeParameters& params)
        {
            return std::unique_ptr<TreeNode>(new T(name, params));
        };
        registerBuilder(ID, builder);
    }

    template <typename T>
    typename std::enable_if< has_default_constructor<T>::value && has_params_constructor<T>::value>::type
    registerNodeTypeImpl(const std::string& ID)
    {
        NodeBuilder builder = [](const std::string& name, const NodeParameters& params)
        {
            if( params.empty() )
            {
                // call this one that MIGHT use default initialization
                return std::unique_ptr<TreeNode>(new T(name));
            }
            return std::unique_ptr<TreeNode>(new T(name, params));
        };
        registerBuilder(ID, builder);
    }


    template<typename T>
    typename std::enable_if< has_static_method_requiredNodeParameters<T>::value>::type
    storeNodeManifest(const std::string& ID)
    {
        manifests_.push_back( { getType<T>(), ID, T::requiredNodeParameters()} );
        sortTreeNodeManifests();
    }

    template<typename T>
    typename std::enable_if< !has_static_method_requiredNodeParameters<T>::value>::type
    storeNodeManifest(const std::string& ID)
    {
        manifests_.push_back( { getType<T>(), ID, NodeParameters()} );
        sortTreeNodeManifests();
    }

    void sortTreeNodeManifests();

    // clang-format on
};

}   // end namespace
#endif   // BT_FACTORY_H
