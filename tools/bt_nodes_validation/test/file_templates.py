from jinja2 import Template


def config_template(
    xml_path: str,
    cpp_dir: str,
    hpp_dir: str,
    base_classes: dict[str, str] = {}
) -> str:
    """Generate configuration file."""
    template = Template("""
        nav2_bt_nodes_file_path: "{{ xml_path }}"
        local_repositories:
          test_repo:
            behavior_trees:
              cpp_dir_paths:
                - "{{ cpp_dir }}"
              hpp_dir_paths:
                - "{{ hpp_dir }}"
              {% if base_classes %}
              hpp_base_classes_paths:
                {% for base_class_name, hpp_base_path in base_classes.items() %}
                - {{ base_class_name }}: "{{ hpp_base_path }}"
                {% endfor %}
              {% endif %}
        """)
    return template.render(
        xml_path=xml_path,
        cpp_dir=cpp_dir,
        hpp_dir=hpp_dir,
        base_classes=base_classes
    )


def xml_node_template(node_type: str, node_id: str, ports: str = '') -> str:
    """Generate XML node definition."""
    template = Template("""
        <{{ node_type }} ID="{{ node_id }}">
            {{ ports }}
        </{{ node_type }}>
    """)
    return template.render(node_type=node_type, node_id=node_id, ports=ports)


def xml_template(xml_nodes: list[str]) -> str:
    """Generate XML file."""
    template = Template("""
        <root BTCPP_format="4">
            <TreeNodesModel>
                {% for xml_node in xml_nodes %}
                    {{ xml_node }}
                {% endfor %}
            </TreeNodesModel>
        </root>
        """)
    return template.render(xml_nodes=xml_nodes)


def cpp_template(class_name: str, node_id: str) -> str:
    """Generate valid CPP registration code."""
    template = Template("""
        #include "test_node.hpp"

        namespace nav2_behavior_tree
        {

        {{ class_name }}::{{ class_name }}(
            const std::string & name,
            const BT::NodeConfiguration & conf)
        : BT::ActionNodeBase(name, conf){}

        inline BT::NodeStatus {{ class_name }}::tick(){
            return BT::NodeStatus::SUCCESS;
        }

        }  // namespace nav2_behavior_tree

        #include "behaviortree_cpp/bt_factory.h"
        BT_REGISTER_NODES(factory)
        {
            factory.registerNodeType<{{ class_name }}>("{{ node_id }}");
        }
    """)
    return template.render(class_name=class_name, node_id=node_id)


def hpp_template(class_name: str, ports: str = '') -> str:
    """Generate valid HPP header code."""
    template = Template("""
        #ifndef TEST_NODE_HPP_
        #define TEST_NODE_HPP_

        #include "behaviortree_cpp/action_node.h"
        #include <string>

        namespace nav2_behavior_tree
        {

        class {{ class_name }} : public BT::ActionNodeBase
        {
        public:
        {{ class_name }}(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts()
        {
            return {
                {{ ports }}
            };
        }

        BT::NodeStatus tick() override {}
        };
        }  // namespace nav2_behavior_tree
        #endif  // TEST_NODE_HPP_
    """)
    return template.render(class_name=class_name, ports=ports)


def cpp_custom_base_template(
    class_name: str,
    base_class_name: str,
    node_id: str
) -> str:
    """Generate valid CPP registration code for node inheriting from custom base."""
    template = Template("""
        #include "test.hpp"

        namespace nav2_behavior_tree
        {

        {{ class_name }}::{{ class_name }}(
          const std::string & name,
          const BT::NodeConfiguration & conf)
        : {{ base_class_name }}<action::type>(name, "test", conf)
        {
        }

        BT::NodeStatus {{ class_name }}::tick()
        {
          return BT::NodeStatus::SUCCESS;
        }

        }  // namespace nav2_behavior_tree

        #include "behaviortree_cpp/bt_factory.h"
        BT_REGISTER_NODES(factory)
        {
        BT::NodeBuilder builder =
            [](const std::string & name, const BT::NodeConfiguration & config)
            {
            return std::make_unique<nav2_behavior_tree::{{ class_name }}>(
                name, "test", config);
            };

        factory.registerBuilder<nav2_behavior_tree::{{ class_name }}>(
            "{{ node_id }}", builder);
        }
    """)
    return template.render(
        class_name=class_name,
        base_class_name=base_class_name,
        node_id=node_id
    )


def hpp_custom_base_template(
    class_name: str,
    base_class_name: str,
    ports: str = ''
) -> str:
    """Generate valid HPP header code for node inheriting from custom base."""
    template = Template("""
        #ifndef TEST_NODE_HPP_
        #define TEST_NODE_HPP_

        #include "bt_custom_node_base.hpp"

        namespace nav2_behavior_tree
        {

        class {{ class_name }} : public {{ base_class_name }}<action::type>
        {
        public:
          {{ class_name }}(
            const std::string & name,
            const std::string & action_name,
            const BT::NodeConfiguration & conf);

          void on_tick() override;

          static BT::PortsList providedPorts()
          {
            return providedBasicPorts(
              {
                {{ ports }};
              });
          }
        };
        }  // namespace nav2_behavior_tree
        #endif  // TEST_NODE_HPP_
    """)
    return template.render(
        class_name=class_name,
        base_class_name=base_class_name,
        ports=ports
    )


def hpp_base_class_template(base_class_name: str, ports: str = '') -> str:
    """Generate valid HPP header code for custom base class."""
    template = Template("""
        #ifndef TEST_NODE_HPP_
        #define TEST_NODE_HPP_

        #include <string>
        #include "behaviortree_cpp/action_node.h"

        namespace nav2_behavior_tree
        {

        template<class Type>
        class {{ base_class_name }} : public BT::ActionNodeBase
        {
        public:
          {{ base_class_name }}(
            const std::string & name,
            const std::string & action_name,
            const BT::NodeConfiguration & conf)
          : BT::ActionNodeBase(name, conf) {}

          {{ base_class_name }} = delete;
          virtual ~{{ base_class_name }}(){}

          static BT::PortsList providedBasicPorts(BT::PortsList addition)
          {
            BT::PortsList basic = {
                {{ ports }}
            };
            basic.insert(addition.begin(), addition.end());
            return basic;
          }

          static BT::PortsList providedPorts()
          {
            return providedBasicPorts({});
          }

          virtual void on_tick(){}

          virtual BT::NodeStatus tick() override = 0;
        };
        }  // namespace nav2_behavior_tree
        #endif  // TEST_NODE_HPP_
    """)
    return template.render(base_class_name=base_class_name, ports=ports)


def cpp_multi_class_template(classes: list[dict[str, str]]) -> str:
    """Generate CPP file with multiple class implementations and registrations."""
    template = Template("""
        #include "test_node.hpp"

        namespace nav2_behavior_tree
        {
        {% for class in classes %}

        {{ class.class_name }}::{{ class.class_name }}(
            const std::string & name,
            const BT::NodeConfiguration & conf)
        : {{ class.base_class_name }}<srv::type>(name, conf){}

        inline void {{ class.class_name }}::on_tick(){}
        {% endfor %}

        }  // namespace nav2_behavior_tree

        #include "behaviortree_cpp/bt_factory.h"
        BT_REGISTER_NODES(factory)
        {
        {% for class in classes %}
            factory.registerNodeType<{{ class.class_name }}>("{{ class.node_id }}");
        {% endfor %}
        }
    """)
    return template.render(classes=classes)


def hpp_multi_class_template(classes: list[dict[str, str]]) -> str:
    """Generate HPP header with multiple class declarations."""
    template = Template("""
        #ifndef TEST_NODE_HPP_
        #define TEST_NODE_HPP_

        #include "behaviortree_cpp/action_node.h"

        namespace nav2_behavior_tree
        {
        {% for class in classes %}

        class {{ class.class_name }} : public {{ class.base_class_name }}<srv::type>
        public:
          {
            {{ class.class_name }}(
              const std::string& name,
              const BT::NodeConfiguration& config);

            static BT::PortsList providedPorts()
            {
              return providedBasicPorts({
                {{ class.ports }}
              });
            }

            void on_tick() override;
          };
        {% endfor %}

        }  // namespace nav2_behavior_tree
        #endif  // TEST_NODE_HPP_
    """)
    return template.render(classes=classes)
