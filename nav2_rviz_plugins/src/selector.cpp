#include "nav2_rviz_plugins/selector.hpp"
#include "rviz_common/display_context.hpp"
#include <QImage> 

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{
Selector::Selector(QWidget * parent)
: Panel(parent)
{
  client_node_ = std::make_shared<rclcpp::Node>("example");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  pub_controller_ = client_node_->create_publisher<std_msgs::msg::String>("controller_selector", qos);

  main_layout = new QVBoxLayout;
  left_layout = new QVBoxLayout;
  right_layout = new QVBoxLayout;
  controller = new QComboBox;
  controller_params = new QComboBox;
  planner = new QComboBox;
  planner_params = new QComboBox;

  image_ = new QImage("nav2_logo.png");
  imgDisplayLabel = new QLabel("");
  imgDisplayLabel->setPixmap(QPixmap::fromImage(*image_));
  imgDisplayLabel->adjustSize();

  main_layout->setContentsMargins(10, 10, 10, 10);
  left_layout->addWidget(imgDisplayLabel);
  left_layout->setContentsMargins(5, 5, 5, 5);
  right_layout->addWidget(new QLabel("Controller"));
  right_layout->addWidget(controller);
  right_layout->addWidget(new QLabel("Planner"));
  right_layout->addWidget(planner);
  right_layout->addWidget(new QLabel("Controller params"));
  right_layout->addWidget(controller_params);
  right_layout->addWidget(new QLabel("Planner params"));
  right_layout->addWidget(planner_params);

  main_layout->addLayout(left_layout);
  main_layout->addLayout(right_layout);

  setLayout(main_layout);
  timer_.start(200, this);

  connect(
    controller, QOverload<int>::of(&QComboBox::activated), this,
    &Selector::setController);

}

Selector::~Selector()
{
}

void Selector::setController()
{
  std_msgs::msg::String msg;
  controller_name_ = controller->currentText().toStdString();
  msg.data = controller_name_;

  pub_controller_->publish(msg);
  timer_.start(200, this);
}

void
Selector::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void
Selector::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_.timerId()) {
    auto parameter_client_con = std::make_shared<rclcpp::SyncParametersClient>(client_node_, "controller_server");
    QStringList id_list;
    
    while (!parameter_client_con->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(client_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
      RCLCPP_INFO(client_node_->get_logger(), "service not available, waiting again...");
    }

    if (!plugins_loaded_) {
      auto parameters_con = parameter_client_con->get_parameters({"controller_plugins"});
      auto str_arr = parameters_con[0].as_string_array();
      for (auto str: str_arr)
        id_list.push_back(QString::fromStdString(str));
      controller->addItems(id_list);
      id_list.clear();
    } else {
      auto parameters_con = parameter_client_con->get_parameters({"FollowPath.plugin"});
      std::cout<<parameters_con.size()<<std::endl;      
      // auto str_arr = parameters_con[0].as_string_array();
      // for (auto str: str_arr)
      //   id_list.push_back(QString::fromStdString(str));
      // controller_params->addItems(id_list);
      // id_list.clear();
    }
    const auto parameters = parameter_client_con->list_parameters({}, 50); // Timeout in seconds

    for (const auto &param_name : parameters.names) {
      std::cout << param_name << std::endl;
    }
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(client_node_, "planner_server");

    while (!parameter_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(client_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(client_node_->get_logger(), "service not available, waiting again...");
    }

    if (!plugins_loaded_) {
      auto parameters = parameter_client->get_parameters({"planner_plugins"});
      auto str_arr1 = parameters[0].as_string_array();
      for (auto str: str_arr1)
        id_list.push_back(QString::fromStdString(str));
      planner->addItems(id_list);
      id_list.clear();
      plugins_loaded_ = true;
    } else {
      auto parameters_con = parameter_client_con->get_parameters({controller_name_});
      // auto str_arr = parameters_con.as_string_array();
      // for (auto str: str_arr)
      //   id_list.push_back(QString::fromStdString(str));
      // controller_params->addItems(id_list);
      // id_list.clear();
    }
    
    timer_.stop();
  }
  // if (controller->count() != 0 && planner->count() != 0)
    // start_ros_timer();
}

void
Selector::start_ros_timer()
{
  rclcpp::spin(node_);
}

} // namespace nav2_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Selector, rviz_common::Panel)