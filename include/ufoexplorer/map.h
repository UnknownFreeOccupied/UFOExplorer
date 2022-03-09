#ifndef UFOEXPLORER_MAP_H
#define UFOEXPLORER_MAP_H

// UFO
#include <ufo/map/occupancy_map.h>
#include <ufo/map/types.h>
#include <ufo/math/vector3.h>
#include <ufoexplorer/model/robot_model.h>
#include <ufoexplorer/model/sensor_model.h>
#include <ufomap_msgs/UFOMapMetaData.h>
#include <ufomap_msgs/UFOMapStamped.h>

// STL
#include <shared_mutex>
#include <string>
#include <variant>

namespace ufoexplorer
{
struct UFOMapMutex {
	std::unique_ptr<ufo::map::OccupancyMapSmall> map;
	mutable std::shared_mutex mutex;
};

std::optional<ufo::geometry::AABB> updateMap(
    UFOMapMutex& map, ufomap_msgs::UFOMapStamped::ConstPtr const& msg,
    bool automatic_pruning, double occupied_thres, double free_thres);

bool isCollisionFreeLine(UFOMapMutex const& map, ufo::math::Vector3 const& start,
                         ufo::math::Vector3 const& end, RobotModel const& robot,
                         bool unknown_as_occupied = true, ufo::map::DepthType depth = 0);

bool inFreeSpace(UFOMapMutex const& map, ufo::math::Vector3 const& point,
                 RobotModel const& robot, bool unknown_as_occupied = true,
                 ufo::map::DepthType depth = 0);

std::pair<double, double> getGain(UFOMapMutex const& map,
                                  ufo::math::Vector3 const& position,
                                  ufo::geometry::AABB boundary, SensorModel const& sensor,
                                  bool continue_outside = true,
                                  ufo::map::DepthType depth = 0);

double getExploredVolume(UFOMapMutex const& map,
                         ufo::geometry::AABB const& updated_volume,
                         ufo::geometry::AABB const& bounding_volume,
                         ufo::map::DepthType depth);
}  // namespace ufoexplorer

#endif  // UFOEXPLORER_MAP_H