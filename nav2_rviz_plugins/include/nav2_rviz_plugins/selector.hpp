#ifndef SELECTOR_HPP_
#define SELECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include <QtWidgets>
#include "vector"
#include "memory"
#include "string"
#include <QBasicTimer>
#include <QFrame>
#include <QGridLayout>
#include <QParallelAnimationGroup>
#include <QScrollArea>
#include <QToolButton>
#include <QWidget>
#include "std_msgs/msg/string.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{
class Selector : public rviz_common::Panel
{
	Q_OBJECT

public:
  explicit Selector(QWidget * parent = 0);
  ~Selector();
  void onInitialize() override;

private:
	void timerEvent(QTimerEvent * event) override;
	// The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_controller_;
  std::string controller_name_;

  bool plugins_loaded_ = false;

  QBasicTimer timer_;
  QImage * image_;
  QLabel * imgDisplayLabel;
  QVBoxLayout * main_layout;
  QVBoxLayout * left_layout;
  QVBoxLayout * right_layout;
  QComboBox * controller;
  QComboBox * planner;
  QComboBox * controller_params;
  QComboBox * planner_params;

  QToolButton toggleButton;
  QScrollArea contentArea;
  QParallelAnimationGroup toggleAnimation;
  int animationDuration{300};
  rclcpp::TimerBase::SharedPtr rclcpp_timer_;
  void timer_callback();
  void start_ros_timer();
  void setController();

protected:
	QVBoxLayout * layout1 = new QVBoxLayout;
// 	QComboBox * combo = new QComboBox;
};

} // namespace nav2_rviz_plugins


#endif  // SELECTOR_HPP_