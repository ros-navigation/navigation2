#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <vector>
#include <string>

/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_route_tool.h>

// Other ROS dependencies
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_route/graph_loader.hpp>
#include <nav2_route/types.hpp>
#include <nav2_route/utils.hpp>


namespace route_tool
{
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class routeTool : public rviz_common::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

        public:
            /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
            routeTool(QWidget * parent = nullptr);

            /**
             *  Now we declare overrides of rviz_common::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
            virtual void save(rviz_common::Config config) const;
            virtual void load(const rviz_common::Config & config);


        /**
         *  Here we declare some internal slots.
         */
        private Q_SLOTS:
            void on_load_button_clicked(void);

            void on_save_button_clicked(void);

            void on_create_button_clicked(void);

            void on_submit_button_clicked(void);

            void on_delete_button_clicked(void);

            void update_display(void);

        Q_SIGNALS:

            void display_changed();

        /**
         *  Finally, we close up with protected member variables
         */
        protected:
            // UI pointer
            std::unique_ptr<Ui::route_tool> ui_;
            // ROS declaration

        private:
            void load_route_graph(std::string filename);
            void save_route_graph(void);
            void load_map(std::string filename);

            nav2_util::LifecycleNode::SharedPtr node_;
            std::shared_ptr<nav2_route::GraphLoader> graph_loader_;
            std::shared_ptr<tf2_ros::Buffer> tf_;
            nav2_route::Graph graph_;
            nav2_route::GraphToIDMap graph_to_id_map_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                graph_vis_publisher_;


    };
} // namespace route_tool
#endif
