/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef MESSAGE_FILTERS_SUBSCRIBER_H
#define MESSAGE_FILTERS_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>

#include "connection.h"
#include "simple_filter.h"

namespace message_filters
{

class SubscriberBase
{
public:
  virtual ~SubscriberBase() {}
  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   * \param callback_queue The callback queue to pass along
   */
//  virtual void subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0) = 0;
  virtual void subscribe(rclcpp::Node* nh, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default) = 0;
  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
   */
  virtual void subscribe() = 0;
  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  virtual void unsubscribe() = 0;
};
typedef std::shared_ptr<SubscriberBase> SubscriberBasePtr;

/**
 * \brief ROS subscription filter.
 *
 * This class acts as a highest-level filter, simply passing messages from a ROS subscription through to the
 * filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * The Subscriber object is templated on the type of message being subscribed to.
 *
 * \section connections CONNECTIONS
 *
 * Subscriber has no input connection.
 *
 * The output connection for the Subscriber object is the same signature as for roscpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const>&);
\endverbatim
 */
template<class M>
class Subscriber : public SubscriberBase, public SimpleFilter<M>
{
public:
//  typedef std::shared_ptr<M const> MConstPtr;
  typedef MessageEvent<M const> EventType;

  /**
   * \brief Constructor
   *
   * See the ros::NodeHandle::subscribe() variants for more information on the parameters
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   * \param callback_queue The callback queue to pass along
   */
  Subscriber(rclcpp::Node* nh, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(nh, topic, qos);
  }

  /**
   * \brief Empty constructor, use subscribe() to subscribe to a topic
   */
  Subscriber()
  {
  }

  ~Subscriber()
  {
    unsubscribe();
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   * \param callback_queue The callback queue to pass along
   */
  void subscribe(rclcpp::Node* nh, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    unsubscribe();

    if (!topic.empty())
    {
      topic_ = topic;
      qos_ = qos;
      sub_ = nh->create_subscription<M>(topic,
               [this](std::shared_ptr<M const> msg) {
                 this->cb(EventType(msg));
               }, qos); 
      nh_ = nh;
    }
  }

  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
   */
  void subscribe()
  {
    unsubscribe();

    if (!topic_.empty())
    {
      subscribe(nh_, topic_, qos_);
    }
  }

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  void unsubscribe()
  {
//    sub_.shutdown();
  }

  std::string getTopic() const
  {
    return this->topic_;
  }

  /**
   * \brief Returns the internal ros::Subscriber object
   */
  const typename rclcpp::Subscription<M>::SharedPtr getSubscriber() const { return sub_; }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  template<typename F>
  void connectInput(F& f)
  {
    (void)f;
  }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  void add(const EventType& e)
  {
    (void)e;
  }

  void cb(const EventType& e)
  {
    this->signalMessage(e);
  }

  typename rclcpp::Subscription<M>::SharedPtr sub_;
  rclcpp::Node* nh_;
  std::string topic_;
  rmw_qos_profile_t qos_;
};

}

#endif
