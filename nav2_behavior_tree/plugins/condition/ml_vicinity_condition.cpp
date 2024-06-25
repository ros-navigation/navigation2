// Copyright (c) 2024 Andy Zelenak
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

#include "nav2_behavior_tree/plugins/condition/ml_vicinity_condition.hpp"
#include <chrono>
#include <cstdlib>
#include <string>
#include <cv_bridge/cv_bridge.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace nav2_behavior_tree
{

MLVicinityCondition::MLVicinityCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  std::string image_topic;
  getInput<std::string>("image_topic", image_topic);
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10, std::bind(&MLVicinityCondition::imageCallback,
      this, std::placeholders::_1));
}

BT::NodeStatus MLVicinityCondition::tick()
{
  if (promptAIModel()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

bool MLVicinityCondition::promptAIModel()
{
  // Get OpenAI key from environment variable
  const char * openai_key = std::getenv("OPENAI_API_KEY");
  RCLCPP_INFO_STREAM(node_->get_logger(), openai_key);

  const std::string base_url = "https://api.openai.com/v1/chat/completions";
  std::string response;
  CURL * curl = curl_easy_init();

  const std::string prompt = "Are there any obstacles in the image which would prevent forward "
    "motion or possibly damage a robot as it moves forward? Note that a small doorsill is not a "
    "significant obstacle. Please answer in one word, yes or no.";

  // Lambda to handle service response
  auto write_callback = [&](void * contents, size_t size, size_t nmemb, std::string * response)
    {
      size_t total_size = size * nmemb;
      response->append(reinterpret_cast<char *>(contents), total_size);
      return total_size;
    };

  if (curl) {
    // See https://drqwrites.medium.com/accessing-the-openai-api-using-c-a3e527b6584b
    nlohmann::json request_data;
    request_data["model"] = "gpt-4o";
    request_data["messages"][0]["role"] = "user";
    // Attach the latest image
    std::string encoded_image = base64_encode(enc_msg, buf.size());
    {
      const std::lock_guard<std::mutex> lock(image_mutex_);
      // If we have no image yet
      if (!latest_image_) {
        return false;
      }
      std::vector<uchar> buf;
      cv::imencode(".jpg", img, buf);
      auto *enc_msg = reinterpret_cast<unsigned char*>(buf.data());
      encoded_image = base64_encode(enc_msg, buf.size());
    }
    request_data["messages"][0]["content"] = {prompt, encoded_image};

    std::string request_data_str = request_data.dump().c_str();

    struct curl_slist * headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers,
        ("Authorization: Bearer " + std::string(openai_key)).c_str());
    curl_easy_setopt(curl, CURLOPT_URL, base_url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_data_str.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, request_data_str.length());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    CURLcode res = curl_easy_perform(curl);

    if (res != CURLE_OK) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Curl request failed: " << curl_easy_strerror(res));
    }

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
  }

  nlohmann::json jresponse = nlohmann::json::parse(response);

  std::string string_response = jresponse["choices"][0]["message"]["content"].get<std::string>();

  // Parse for a one-word Yes/No reply
  // Yes means there is an obstacle
  if ((string_response.find("Yes") != std::string::npos) ||
    (string_response.find("yes") != std::string::npos))
  {
    return false;
  }
  return true;
}
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::MLVicinityCondition>("MLVicinityCondition");
}
