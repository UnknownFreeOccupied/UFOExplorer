// UFO
#include <ufoexplorer/map.h>
#include <ufomap_msgs/conversions.h>

// STL
#include <algorithm>

namespace ufoexplorer
{
std::optional<ufo::geometry::AABB> updateMap(
    UFOMapMutex& map, ufomap_msgs::UFOMapStamped::ConstPtr const& msg,
    bool automatic_pruning, double occupied_thres, double free_thres)
{
	if (!map.map) {
		map.map.reset(new ufo::map::OccupancyMapSmall(msg->map.info.resolution,
		                                              msg->map.info.depth_levels));
	}
	return ufomap_msgs::msgToUfo(msg->map, *map.map);
}

bool isCollisionFreeLine(UFOMapMutex const& map, ufo::math::Vector3 const& start,
                         ufo::math::Vector3 const& end, RobotModel const& robot,
                         bool unknown_as_occupied, ufo::map::DepthType depth)
{
	std::shared_lock read_lock(map.mutex);

	if (!map.map) {
		return false;
	}

	ufo::map::Point3 direction = end - start;
	ufo::map::Point3 center = start + (direction / 2.0);
	double distance = direction.norm();
	direction /= distance;

	double yaw = -atan2(direction[1], direction[0]);
	double pitch = -asin(direction[2]);
	double roll = 0;  // TODO: Fix

	ufo::geometry::OBB obb(center,
	                       ufo::map::Point3(distance / 2.0, robot.radius, robot.height),
	                       ufo::math::Quaternion(roll, pitch, yaw));

	auto pred = ufo::map::predicate::LeafNode(depth) &&
	            ufo::map::predicate::OccupancyStates(true, false, unknown_as_occupied) &&
	            ufo::map::predicate::Intersects(obb);
	for (auto it = map.map->beginQuery(pred); it != map.map->endQuery(); ++it) {
		return false;
	}
	return true;
}

bool inFreeSpace(UFOMapMutex const& map, ufo::math::Vector3 const& point,
                 RobotModel const& robot, bool unknown_as_occupied,
                 ufo::map::DepthType depth)
{
	std::shared_lock read_lock(map.mutex);

	if (!map.map) {
		return false;
	}

	double res = map.map->getResolution();
	// Expand with res to get a buffer
	ufo::map::Point3 min(point[0] - robot.radius - res, point[1] - robot.radius - res,
	                     point[2] - (robot.height / 2.0) - res);
	ufo::map::Point3 max(point[0] + robot.radius + res, point[1] + robot.radius + res,
	                     point[2] + (robot.height / 2.0) + res);

	ufo::geometry::AABB aabb(min, max);

	auto pred = ufo::map::predicate::LeafNode(depth) &&
	            ufo::map::predicate::OccupancyStates(true, false, unknown_as_occupied) &&
	            ufo::map::predicate::Intersects(aabb);

	for (auto it = map.map->beginQuery(pred); it != map.map->endQuery(); ++it) {
		return false;
	}
	return true;
}

std::pair<double, double> getGain(UFOMapMutex const& map,
                                  ufo::math::Vector3 const& position,
                                  ufo::geometry::AABB boundary, SensorModel const& sensor,
                                  bool continue_outside, ufo::map::DepthType depth)
{
	std::shared_lock read_lock(map.mutex);

	if (!map.map) {
		return {0.0, 0.0};
	}

	// This function computes the gain
	double fov_y = sensor.horizontal_fov * 180.0 / M_PI,
	       fov_p = sensor.vertical_fov * 180.0 / M_PI;

	double dr = map.map->getNodeSize(depth), dphi = 10, dtheta = 10;
	double dphi_rad = M_PI * dphi / 180.0, dtheta_rad = M_PI * dtheta / 180.0;
	double r;
	int phi, theta;
	double phi_rad, theta_rad;

	int canvas_width = 360 / dtheta;
	int canvas_height = fov_p / dphi;

	std::vector<std::vector<double>> gain_image(canvas_width,
	                                            std::vector<double>(canvas_height, 0.0));
	std::vector<std::vector<int>> occupied_image(canvas_width,
	                                             std::vector<int>(canvas_height, 0));

	for (int i = 0, theta = 0; theta < 360 && i < canvas_width; theta += dtheta, ++i) {
		theta_rad = M_PI * theta / 180.0;
		for (int j = 0, phi = 90 - (fov_p / 2); phi < 90 + (fov_p / 2) && j < canvas_height;
		     phi += dphi, ++j) {
			phi_rad = M_PI * phi / 180.0;

			double g = 0;
			for (r = 0.0; r < sensor.range_max; r += dr) {
				ufo::math::Vector3 vec(position[0] + r * cos(theta_rad) * sin(phi_rad),
				                       position[1] + r * sin(theta_rad) * sin(phi_rad),
				                       position[2] + r * cos(phi_rad));

				if (!continue_outside && !ufo::geometry::intersects(boundary, vec)) {
					// TODO: Add as an option to calculate gain outside of boundary
					break;
				}

				double occ = map.map->getOccupancy(map.map->toCode(vec, depth));
				if (map.map->getOccupiedThres() < occ) {
					// Break if occupied so we do not count any information gain behind a
					// wall
					occupied_image[i][j] = 1;
					break;
				} else if (map.map->getFreeThres() <= occ) {
					if (sensor.range_min <= r) {
						gain_image[i][j] += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad *
						                    sin(phi_rad) * sin(dphi_rad / 2);
					}
				}
			}
		}
	}

	std::vector<double> gain_histogram(gain_image.size(), 0.0);
	std::vector<int> occupied_histogram(occupied_image.size(), 0);
	for (size_t i = 0; i < gain_image.size(); ++i) {
		for (double g : gain_image[i]) {
			gain_histogram[i] += g;
		}
		for (int g : occupied_image[i]) {
			if (0 < g) {
				occupied_histogram[i] = 1;
				break;
			}
		}
	}

	// Moving window
	int window_size = gain_histogram.size() * (sensor.horizontal_fov / (2.0 * M_PI));
	int half_window_size = window_size / 2;
	int best_view = half_window_size;
	double best_gain =
	    std::accumulate(gain_histogram.begin(), gain_histogram.begin() + window_size, 0.0);
	double current_gain = best_gain;
	int current_occupied = std::accumulate(occupied_histogram.begin(),
	                                       occupied_histogram.begin() + window_size, 1);
	if (0 < current_occupied) {
		best_gain *= 10.0;
	}
	for (int i = 0; i < gain_histogram.size(); ++i) {
		current_gain -= gain_histogram[i];
		current_occupied -= occupied_histogram[i];
		current_gain += gain_histogram[(window_size + i) % gain_histogram.size()];
		current_occupied += occupied_histogram[(window_size + i) % occupied_histogram.size()];
		double temp_gain = current_gain;
		if (0 < current_occupied) {
			temp_gain *= 10.0;
		}
		if (temp_gain > best_gain) {
			best_gain = temp_gain;
			best_view = half_window_size + i + 1;
		}
	}

	// bool has_occupied = false;
	// for (int i = best_view - half_window_size; i < best_view + half_window_size;
	//      ++i) {
	//   int index = i;
	//   if (0 > index) {
	//     index = gain_histogram.size() - index;
	//   } else if (occupied_histogram.size() <= index) {
	//     index = index - gain_histogram.size();
	//   }
	//   if (occupied_histogram[index]) {
	//     has_occupied = true;
	//     break;
	//   }
	// }

	// Volume of pyramid
	double h_max = 2.0 * sensor.range_max * tan(sensor.horizontal_fov / 2.0);
	double v_max = 2.0 * sensor.range_max * tan(sensor.vertical_fov / 2.0);
	double max_potential = (sensor.range_max * h_max * v_max) / 3.0;

	// Remove range_min
	h_max = 2.0 * sensor.range_min * tan(sensor.horizontal_fov / 2.0);
	v_max = 2.0 * sensor.range_min * tan(sensor.vertical_fov / 2.0);
	max_potential -= (sensor.range_min * h_max * v_max) / 3.0;

	double gain = best_gain / max_potential;
	double yaw =
	    (static_cast<double>(best_view % gain_histogram.size()) / gain_histogram.size()) *
	    2.0 * M_PI;

	// if (has_occupied) {
	//   return {gain * 10.0, yaw};
	// }

	return {gain, yaw};
}

double getExploredVolume(UFOMapMutex const& map,
                         ufo::geometry::AABB const& updated_volume,
                         ufo::geometry::AABB const& bounding_volume,
                         ufo::map::DepthType depth)
{
	ufo::math::Vector3 updated_volume_min = updated_volume.getMin();
	ufo::math::Vector3 updated_volume_max = updated_volume.getMax();
	ufo::math::Vector3 bounding_volume_min = bounding_volume.getMin();
	ufo::math::Vector3 bounding_volume_max = bounding_volume.getMax();

	ufo::math::Vector3 updated_bounded_volume_min;
	ufo::math::Vector3 updated_bounded_volume_max;
	for (int i : {0, 1, 2}) {
		updated_bounded_volume_min[i] =
		    std::max(updated_volume_min[i], bounding_volume_min[i]);
		updated_bounded_volume_max[i] =
		    std::min(updated_volume_max[i], bounding_volume_max[i]);
	}

	ufo::geometry::AABB aabb(updated_bounded_volume_min, updated_bounded_volume_max);

	std::shared_lock read_lock(map.mutex);

	if (!map.map) {
		return 0.0;
	}

	ufo::math::Vector3 min = aabb.getMin();
	ufo::math::Vector3 max = aabb.getMax();
	double explored = 0.0;

	auto pred =
	    ufo::map::predicate::LeafNode(depth) && ufo::map::predicate::Intersects(aabb);
	for (auto it = map.map->beginQuery(pred); it != map.map->endQuery(); ++it) {
		ufo::geometry::AAEBB bbx = it->getBoundingVolume();
		ufo::math::Vector3 bbx_min = bbx.getMin();
		ufo::math::Vector3 bbx_max = bbx.getMax();

		for (int i : {0, 1, 2}) {
			bbx_min[i] = std::max(min[i], bbx_min[i]);
			bbx_max[i] = std::min(max[i], bbx_max[i]);
		}
		double volume =
		    (bbx_max[0] - bbx_min[0]) * (bbx_max[1] - bbx_min[1]) * (bbx_max[2] - bbx_min[2]);

		if (!map.map->isUnknown(*it)) {
			explored += volume;
		}
	}
	return explored;
}
}  // namespace ufoexplorer