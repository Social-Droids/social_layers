#ifndef SOCIAL_LAYERS_PEOPLE_LAYER_H
#define SOCIAL_LAYERS_PEOPLE_LAYER_H

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <dynamic_reconfigure/server.h>
#include <social_layers/PeopleLayerConfig.h>

#include <social_msgs/People.h>
#include <social_msgs/Person.h>
#include <boost/thread.hpp>

#include <list>

namespace social_layers
{
  class PeopleLayer : public costmap_2d::Layer
  {
  public:
    PeopleLayer();
    bool isDiscretized();

    virtual void onInitialize();
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
      int min_i, int min_j,
      int max_i, int max_j);
    virtual void updateBounds(
      double origin_x, double origin_y, double origin_yaw,
      double* min_x, double* min_y,
      double* max_x, double* max_y);
    virtual void updateBoundsFromPeople(
      double* min_x, double* min_y,
      double* max_x, double* max_y);


  private:
    void configure(PeopleLayerConfig &config, uint32_t level);
    dynamic_reconfigure::Server<PeopleLayerConfig>* server_;
    dynamic_reconfigure::Server<PeopleLayerConfig>::CallbackType f_;

    social_msgs::People people_list_;
    std::list<social_msgs::Person> transformed_people_;

    ros::Subscriber people_sub_;
    void peopleCallback(const social_msgs::People& people);

    double cutoff_, amplitude_, covariance_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    boost::recursive_mutex lock_;
    bool first_time_;

  };

}  // namespace social_layers
#endif  // SOCIAL_LAYERS_PEOPLE_LAYER_H
