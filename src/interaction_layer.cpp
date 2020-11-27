#include <social_layers/interaction_layer.h>
#include <social_layers/util.h>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <string>

PLUGINLIB_EXPORT_CLASS(social_layers::InteractionLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_layers
{

  InteractionLayer::InteractionLayer()
  {

  }

  bool InteractionLayer::isDiscretized()
  {
    return false;
  }

  void InteractionLayer::configure(InteractionLayerConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
    cutoff_ = config.cutoff;
    amplitude_ = config.amplitude;
    covariance_ = config.covariance;
  }

  void InteractionLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    first_time_ = true;
    interaction_sub_ = nh.subscribe("/points_interaction", 1, &InteractionLayer::interactionCallback, this);

    server_ = new dynamic_reconfigure::Server<InteractionLayerConfig>(nh);
    f_ = boost::bind(&InteractionLayer::configure, this, _1, _2);
    server_->setCallback(f_);
  }

  void InteractionLayer::interactionCallback(const social_msgs::PointArray& points)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    point_list_ = points;
  }

  void InteractionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_) return;

    if (point_list_.points.size() == 0)
      return;

    if (cutoff_ >= amplitude_)
      return;

    std::list<geometry_msgs::Point>::iterator o_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for (o_it = transformed_points_.begin(); o_it != transformed_points_.end(); ++o_it)
    {
      geometry_msgs::Point point_interaction = *o_it;

      // double angle = atan2(object.velocity.y, object.velocity.x);
      double angle = 0;
      // double mag = sqrt(pow(object.velocity.x, 2) + pow(object.velocity.y, 2));
      double mag = 0;
      double factor = 1.0 + mag;
      double base = Util().get_radius(cutoff_, amplitude_, covariance_);
      double point = Util().get_radius(cutoff_, amplitude_, covariance_ * factor);

      unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                   height = std::max(1, static_cast<int>((base + point) / res));

      double cx = point_interaction.x, cy = point_interaction.y;

      double ox, oy;
      if (sin(angle) > 0)
        oy = cy - base;
      else
        oy = cy + (point - base) * sin(angle) - base;
      if (cos(angle) >= 0)
        ox = cx - base;
      else
        ox = cx + (point - base) * cos(angle) - base;


      int dx, dy;
      costmap->worldToMapNoBounds(ox, oy, dx, dy);

      int start_x = 0, start_y = 0, end_x = width, end_y = height;
      if (dx < 0)
        start_x = -dx;
      else if (dx + width > costmap->getSizeInCellsX())
        end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

      if (static_cast<int>(start_x + dx) < min_i)
        start_x = min_i - dx;
      if (static_cast<int>(end_x + dx) > max_i)
        end_x = max_i - dx;

      if (dy < 0)
        start_y = -dy;
      else if (dy + height > costmap->getSizeInCellsY())
        end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

      if (static_cast<int>(start_y + dy) < min_j)
        start_y = min_j - dy;
      if (static_cast<int>(end_y + dy) > max_j)
        end_y = max_j - dy;

      double bx = ox + res / 2,
             by = oy + res / 2;
      for (int i = start_x; i < end_x; i++)
      {
        for (int j = start_y; j < end_y; j++)
        {
          unsigned char old_cost = costmap->getCost(i + dx, j + dy);
          if (old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx + i * res, y = by + j * res;
          double ma = atan2(y - cy, x - cx);
          double diff = angles::shortest_angular_distance(angle, ma);
          double a;
          if (fabs(diff) < M_PI / 2)
            a = Util().gaussian(x, y, cx, cy, amplitude_, covariance_ * factor, covariance_, angle);
          else
            a = Util().gaussian(x, y, cx, cy, amplitude_, covariance_,          covariance_, 0);

          if (a < cutoff_)
            continue;
          unsigned char cvalue = (unsigned char) a;
          costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        }
      }
    }
  }

  void InteractionLayer::updateBounds(double origin_x, double origin_y, double origin_z,
                                 double* min_x, double* min_y,
                                 double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    std::string global_frame = layered_costmap_->getGlobalFrameID();
    transformed_points_.clear();

    for (unsigned int i = 0; i < point_list_.points.size(); i++)
    {
      geometry_msgs::Point& point_interaction = point_list_.points[i];
      geometry_msgs::Point tpt;
      geometry_msgs::PointStamped pt, opt;

      try
      {
        pt.point = point_interaction;
        pt.header.frame_id = "map"; //objects_list_.header.frame_id;
        // pt.header.stamp = objects_list_.header.stamp;
        tf_->transform(pt, opt, global_frame);
        tpt = opt.point;

        // pt.point.x += object.velocity.x;
        // pt.point.y += object.velocity.y;
        // pt.point.z += object.velocity.z;
        // tf_->transform(pt, opt, global_frame);

        // tpt.velocity.x = opt.point.x - tpt.position.x;
        // tpt.velocity.y = opt.point.y - tpt.position.y;
        // tpt.velocity.z = opt.point.z - tpt.position.z;

        transformed_points_.push_back(tpt);
      }
      catch (tf2::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        continue;
      }
      catch (tf2::ConnectivityException& ex)
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        continue;
      }
      catch (tf2::ExtrapolationException& ex)
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        continue;
      }
    }

    updateBoundsFromObjects(min_x, min_y, max_x, max_y);
    if (first_time_)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      first_time_ = false;
    }
    else
    {
      double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
      *min_x = std::min(last_min_x_, *min_x);
      *min_y = std::min(last_min_y_, *min_y);
      *max_x = std::max(last_max_x_, *max_x);
      *max_y = std::max(last_max_y_, *max_y);
      last_min_x_ = a;
      last_min_y_ = b;
      last_max_x_ = c;
      last_max_y_ = d;
    }
  }

  void InteractionLayer::updateBoundsFromObjects(double* min_x, double* min_y, double* max_x, double* max_y)
  {
    std::list<geometry_msgs::Point>::iterator o_it;

    for (o_it = transformed_points_.begin(); o_it != transformed_points_.end(); ++o_it)
    {
      geometry_msgs::Point point_interaction = *o_it;

      // double mag = sqrt(pow(object.velocity.x, 2) + pow(object.velocity.y, 2));
      double mag = 0;
      double factor = 1.0 + mag;
      double point = Util().get_radius(cutoff_, amplitude_, covariance_ * factor);

      *min_x = std::min(*min_x, point_interaction.x - point);
      *min_y = std::min(*min_y, point_interaction.y - point);
      *max_x = std::max(*max_x, point_interaction.x + point);
      *max_y = std::max(*max_y, point_interaction.y + point);

    }
  }

};  // namespace social_layers
