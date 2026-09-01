// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace
{

struct WorkerState
{
  std::mutex mutex;
  std::condition_variable cv;
  bool started{false};
  bool finished{false};
  bool saw_stop_requested{false};
};

class BlockingActionServer : public TestActionServer<test_msgs::action::Fibonacci>
{
public:
  explicit BlockingActionServer(std::shared_ptr<WorkerState> worker_state)
  : TestActionServer("blocking_fibonacci"),
    worker_state_(std::move(worker_state))
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>) override
  {
    {
      std::lock_guard<std::mutex> lock(worker_state_->mutex);
      worker_state_->started = true;
    }
    worker_state_->cv.notify_all();

    std::unique_lock<std::mutex> lock(worker_state_->mutex);
    while (!isStopRequested()) {
      worker_state_->cv.wait_for(lock, 10ms);
    }

    worker_state_->saw_stop_requested = true;
    worker_state_->finished = true;
    lock.unlock();
    worker_state_->cv.notify_all();
  }

private:
  std::shared_ptr<WorkerState> worker_state_;
};

TEST(TestActionServerThreading, destructor_requests_stop_and_joins_workers)
{
  auto worker_state = std::make_shared<WorkerState>();
  auto server = std::make_shared<BlockingActionServer>(worker_state);
  auto client_node = std::make_shared<rclcpp::Node>("test_action_server_client");
  auto client = rclcpp_action::create_client<test_msgs::action::Fibonacci>(
    client_node, "blocking_fibonacci");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  ASSERT_TRUE(client->wait_for_action_server(2s));

  std::promise<void> goal_accepted_promise;
  auto goal_accepted_future = goal_accepted_promise.get_future();
  auto send_goal_options =
    rclcpp_action::Client<test_msgs::action::Fibonacci>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [&goal_accepted_promise](
    const rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::SharedPtr goal_handle)
    {
      if (goal_handle != nullptr) {
        goal_accepted_promise.set_value();
      }
    };

  test_msgs::action::Fibonacci::Goal goal;
  goal.order = 10;
  client->async_send_goal(goal, send_goal_options);

  ASSERT_EQ(goal_accepted_future.wait_for(2s), std::future_status::ready);

  {
    std::unique_lock<std::mutex> lock(worker_state->mutex);
    ASSERT_TRUE(worker_state->cv.wait_for(lock, 2s, [&worker_state]() {
      return worker_state->started;
    }));
  }

  executor.cancel();
  spin_thread.join();
  executor.remove_node(server);
  executor.remove_node(client_node);

  server.reset();

  std::unique_lock<std::mutex> lock(worker_state->mutex);
  EXPECT_TRUE(worker_state->finished);
  EXPECT_TRUE(worker_state->saw_stop_requested);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
