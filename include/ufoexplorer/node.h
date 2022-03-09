#ifndef UFOEXPLORER_NODE_H
#define UFOEXPLORER_NODE_H

// UFO
#include <ufo/math/vector3.h>

// STL
#include <memory>
#include <unordered_set>

namespace ufoexplorer
{
int const UNDEFINED_CLUSTER = 0;
int const NOISE_CLUSTER = -1;

struct Node : public std::enable_shared_from_this<Node> {
	ufo::math::Vector3 position;
	double yaw;
	double gain;
	int cluster = UNDEFINED_CLUSTER;
	std::unordered_set<std::shared_ptr<Node>> neighbors;

	bool addNeighbor(std::shared_ptr<Node> const& new_neighbor)
	{
		if (new_neighbor) {
			return neighbors.insert(new_neighbor).second;
		}
		return false;
	}

	bool eraseNeighbor(std::shared_ptr<Node> const& neighbor)
	{
		return neighbors.erase(neighbor) > 0;
	}

	double squaredDistance(std::shared_ptr<Node> const& other) const
	{
		// double sqrt_dist_xy =
		//     (position.x() * other->position.x()) + (position.y() * other->position.y());

		// double sqrt_dist_z = position.z() * other->position.z();

		// return sqrt_dist_xy + (2.0 * sqrt_dist_z);

		// double diff = std::max(0.0, sqrt_dist_z - sqrt_dist_xy);

		// return sqrt_dist_xy + ((1.0 + diff) * sqrt_dist_z);

		return (position - other->position).squaredNorm();
	}

	double distance(std::shared_ptr<Node> const& other) const
	{
		return sqrt(squaredDistance(other));
	}
};
}  // namespace ufoexplorer

#endif  // UFOEXPLORER_NODE_H