#ifndef NAV2_COSTMAP_2D__OBJECT_BUFFER_HPP_
#define NAV2_COSTMAP_2D__OBJECT_BUFFER_HPP_

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_costmap_2d
{
  using rclcpp::Time;

  template<class ObjectT>
  struct TimedObject
  {
    ObjectT object;
    Time time;
  };
/**
 * @class ObjectBuffer
 * @brief Class to orderly store Objects for a given period of time
 */
template<class ObjectT>    
class ObjectBuffer
{
    public:
     /**
     * @brief  Constructs an Object buffer.
     * @param  Object_keep_time Defines the persistence of Objects in seconds, 0 means only keep the latest
     */
    ObjectBuffer(
        const nav2_util::LifecycleNode::WeakPtr & parent,
        double object_keep_time
    ):
    object_keep_time_(rclcpp::Duration::from_seconds(object_keep_time))
   {
       auto node = parent.lock();
        clock_ = node->get_clock();
        logger_ = node->get_logger();
   }

    /**
   * @brief  Destructor... cleans up
   */
  ~ObjectBuffer(){};

  /**
   * @brief  stores a Object with an associated generation time
   */
  void bufferObject(const ObjectT & object, const Time & generation_time)
  {
    TimedObject<ObjectT> timed_object;
    timed_object.object = object;
    timed_object.time = generation_time;
    objects_list_.push_front(timed_object);
  }

  /**
   * @brief  Pushes copies of all current Objects onto the end of the vector passed in
   * @param  Objects The vector to be filled
   */
  void getObjects(std::vector<ObjectT> & objects)
  {
    // first... let's make sure that we don't have any stale objects
    purgeStaleObjects();
    // now we'll just copy the objects for the caller
    for (auto obs_it = objects_list_.begin(); obs_it != objects_list_.end(); ++obs_it) {
      objects.push_back(obs_it->object);
    }
    objects_list_.clear();
  }


  private:
  /**
   * @brief  Removes any stale objects from the buffer list
   */
  void purgeStaleObjects()
  {
    if(!objects_list_.empty())
    {
      // if we're keeping objects for no time... then we'll only keep one object
      auto obj_it = objects_list_.begin();
      if(object_keep_time_ == rclcpp::Duration::from_seconds(0.0)){
        objects_list_.erase(++obj_it, objects_list_.end());
        return;
      }

      // otherwise... we'll have to loop through the objectss to see which ones are stale
      for (obj_it = objects_list_.begin(); obj_it != objects_list_.end(); ++obj_it) 
      {
        // check if the objects is out of date... and if it is,
        // remove it and those that follow from the list
        if ((clock_->now() - obj_it->time) > object_keep_time_)
        {
          objects_list_.erase(obj_it, objects_list_.end());
          return;
        }
      }
    }
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};
  const rclcpp::Duration object_keep_time_;
  std::list<TimedObject<ObjectT>> objects_list_;
};

} // namespace nav2_costmap_2d



#endif  // NAV2_COSTMAP_2D__OBJECT_BUFFER_HPP_
