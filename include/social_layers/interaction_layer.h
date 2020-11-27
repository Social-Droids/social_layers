#ifndef SOCIAL_LAYERS_INTERACTION_LAYER_H
#define SOCIAL_LAYERS_INTERACTION_LAYER_H

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <dynamic_reconfigure/server.h>
#include <social_layers/InteractionLayerConfig.h>

#include <social_msgs/PointArray.h>
// #include <social_msgs/Object.h>
#include <boost/thread.hpp>

#include <list>

namespace social_layers
{
  class InteractionLayer : public costmap_2d::Layer
  {
  public:
    InteractionLayer();
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
    void configure(InteractionLayerConfig &config, uint32_t level);
    dynamic_reconfigure::Server<InteractionLayerConfig>* server_;
    dynamic_reconfigure::Server<InteractionLayerConfig>::CallbackType f_;

    social_msgs::PointArray point_list_;
    std::list<geometry_msgs::Point> transformed_points_;

    ros::Subscriber interaction_sub_;
    void interactionCallback(const social_msgs::PointArray& points);

    double cutoff_, amplitude_, covariance_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    boost::recursive_mutex lock_;
    bool first_time_;

  };

}  // namespace social_layers
#endif  // SOCIAL_LAYERS_INTERACTION_LAYER_H
