/*
 * Copyright 2023 Sathakkadhullah.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Obstacle distance 3D plugin
 * @file obstacle_distance_3d.cpp
 * @author Sathakkadhullah <sathak0730@@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/obstacle_distance3_d_list.hpp"

namespace mavros
{
  namespace extra_plugins
  {
    using namespace std::placeholders; // NOLINT

    //! Radians to degrees
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    //! Mavlink MAV_DISTANCE_SENSOR enumeration
    using mavlink::common::MAV_DISTANCE_SENSOR;

    /**
     * @brief Obstacle distance plugin
     * @plugin obstacle_distance
     *
     * Publishes obstacle distance 3D array to the FCU, in order to assist in an obstacle
     * avoidance flight.
     * @see obstacle_cb()
     */
    class ObstacleDistance3DPlugin : public plugin::Plugin
    {
    public:
      explicit ObstacleDistance3DPlugin(plugin::UASPtr uas_)
          : Plugin(uas_, "obstacle_3d")
      {
        enable_node_watch_parameters();
        obstacle_sub =
            node->create_subscription<mavros_msgs::msg::ObstacleDistance3DList>(
                "~/send", 10,
                std::bind(&ObstacleDistance3DPlugin::obstacle_cb, this, _1));
      }

      Subscriptions get_subscriptions() override
      {
        return {/* Rx disabled */};
      }

    private:
      rclcpp::Subscription<mavros_msgs::msg::ObstacleDistance3DList>::SharedPtr obstacle_sub;

      // mavlink::common::MAV_FRAME frame;

      /**
       * @brief Send obstacle distance array to the FCU.
       *
       * Message specification: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
       * @param req	received ObstacleDistance msg
       */
      void obstacle_cb(const mavros_msgs::msg::ObstacleDistance3DList::SharedPtr req)
      {
        for (auto it = req->obstacles.begin(); it != req->obstacles.end(); it++)
        {
          mavlink::ardupilotmega::msg::OBSTACLE_DISTANCE_3D obstacle{};
          obstacle.time_boot_ms = get_time_boot_ms(it->header.stamp);          //!< [millisecs]
          obstacle.sensor_type = utils::enum_value(MAV_DISTANCE_SENSOR::LASER); //!< defaults is laser
          obstacle.min_distance = it->min_distance;                            //!< [meters]
          obstacle.max_distance = it->max_distance;                            //!< [meters]
          obstacle.frame =12;                                                   // utils::enum_value(frame);
          obstacle.obstacle_id = UINT16_MAX;
          obstacle.x = it->position.x; //!< [degrees]
          obstacle.y = it->position.y;
          obstacle.z = it->position.z;           
          uas->send_message(obstacle);
        }
      }
    };
  } // namespace extra_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp> // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::ObstacleDistance3DPlugin)
