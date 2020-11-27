#ifndef SOCIAL_LAYERS_OBJECTS_LAYER_H
#define SOCIAL_LAYERS_OBJECTS_LAYER_H

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <dynamic_reconfigure/server.h>
#include <social_layers/ObjectsLayerConfig.h>

#include <social_msgs/Objects.h>
#include <social_msgs/Object.h>
#include <boost/thread.hpp>

#include <list>

namespace social_layers
{
  class ObjectsLayer : public costmap_2d::Layer
  {
  public:
    ObjectsLayer();
    bool isDiscretized();

    virtual void onInitialize();
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
      int min_i, int min_j,
      int max_i, int max_j);
    virtual void updateBounds(
      double origin_x, double origin_y, double origin_yaw,
      double* min_x, double* min_y,
      double* max_x, double* max_y);
    virtual void updateBoundsFromObjects(
      double* min_x, double* min_y,
      double* max_x, double* max_y);


  private:
    void configure(ObjectsLayerConfig &config, uint32_t level);
    dynamic_reconfigure::Server<ObjectsLayerConfig>* server_;
    dynamic_reconfigure::Server<ObjectsLayerConfig>::CallbackType f_;

    social_msgs::Objects objects_list_;
    std::list<social_msgs::Object> transformed_objects_;

    ros::Subscriber objects_sub_;
    void objectsCallback(const social_msgs::Objects& objects);

    double cutoff_, amplitude_, covariance_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    boost::recursive_mutex lock_;
    bool first_time_;

  };

}  // namespace social_layers
#endif  // SOCIAL_LAYERS_OBJECTS_LAYER_H
