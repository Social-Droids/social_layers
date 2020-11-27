#include <social_layers/people_layer.h>
#include <social_layers/util.h>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <string>

PLUGINLIB_EXPORT_CLASS(social_layers::PeopleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_layers
{

  PeopleLayer::PeopleLayer()
  {

  }

  bool PeopleLayer::isDiscretized()
  {
    return false;
  }

  void PeopleLayer::configure(PeopleLayerConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
    cutoff_ = config.cutoff;
    amplitude_ = config.amplitude;
    covariance_ = config.covariance;
  }

  void PeopleLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    first_time_ = true;
    people_sub_ = nh.subscribe("/people", 1, &PeopleLayer::peopleCallback, this);

    server_ = new dynamic_reconfigure::Server<PeopleLayerConfig>(nh);
    f_ = boost::bind(&PeopleLayer::configure, this, _1, _2);
    server_->setCallback(f_);
  }

  void PeopleLayer::peopleCallback(const social_msgs::People& people)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    people_list_ = people;
  }

  void PeopleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_) return;

    if (people_list_.people.size() == 0)
      return;

    if (cutoff_ >= amplitude_)
      return;

    std::list<social_msgs::Person>::iterator p_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
    {
      social_msgs::Person person = *p_it;

      // double angle = atan2(person.velocity.y, person.velocity.x);
      double angle = 0;
      // double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
      double mag = 0;
      double factor = 1.0 + mag;
      double base = Util().get_radius(cutoff_, amplitude_, covariance_);
      double point = Util().get_radius(cutoff_, amplitude_, covariance_ * factor);

      unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                   height = std::max(1, static_cast<int>((base + point) / res));

      double cx = person.pose.position.x, cy = person.pose.position.y;

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

  void PeopleLayer::updateBounds(double origin_x, double origin_y, double origin_z,
                                 double* min_x, double* min_y,
                                 double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    std::string global_frame = layered_costmap_->getGlobalFrameID();
    transformed_people_.clear();

    for (unsigned int i = 0; i < people_list_.people.size(); i++)
    {
      social_msgs::Person& person = people_list_.people[i];
      social_msgs::Person tpt;
      geometry_msgs::PointStamped pt, opt;

      try
      {
        pt.point = person.pose.position;
        pt.header.frame_id = people_list_.header.frame_id;
        pt.header.stamp = people_list_.header.stamp;
        tf_->transform(pt, opt, global_frame);
        tpt.pose.position = opt.point;

        // pt.point.x += person.velocity.x;
        // pt.point.y += person.velocity.y;
        // pt.point.z += person.velocity.z;
        tf_->transform(pt, opt, global_frame);

        // tpt.velocity.x = opt.point.x - tpt.position.x;
        // tpt.velocity.y = opt.point.y - tpt.position.y;
        // tpt.velocity.z = opt.point.z - tpt.position.z;

        transformed_people_.push_back(tpt);
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

    updateBoundsFromPeople(min_x, min_y, max_x, max_y);
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

  void PeopleLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
  {
    std::list<social_msgs::Person>::iterator p_it;

    for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
    {
      social_msgs::Person person = *p_it;

      // double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
      double mag = 0;
      double factor = 1.0 + mag;
      double point = Util().get_radius(cutoff_, amplitude_, covariance_ * factor);

      *min_x = std::min(*min_x, person.pose.position.x - point);
      *min_y = std::min(*min_y, person.pose.position.y - point);
      *max_x = std::max(*max_x, person.pose.position.x + point);
      *max_y = std::max(*max_y, person.pose.position.y + point);

    }
  }

};  // namespace social_layers
