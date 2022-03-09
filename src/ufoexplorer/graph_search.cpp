// UFO
#include <ufoexplorer/graph_search.h>

// Boost
#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

// STL
#include <queue>
#include <unordered_map>

namespace ufoexplorer
{
namespace bg = boost::geometry;
namespace bgi = bg::index;

// R-tree
using Point = bg::model::point<double, 3, bg::cs::cartesian>;
using Box = bg::model::box<Point>;
using Value = std::pair<Point, std::shared_ptr<Node>>;
using PointRtree = bgi::rtree<Point, bgi::rstar<16>>;
using ValueRtree = bgi::rtree<Value, bgi::rstar<16>>;

using OpenSet =
    std::priority_queue<std::pair<double, std::shared_ptr<Node>>,
                        std::vector<std::pair<double, std::shared_ptr<Node>>>,
                        std::greater<std::pair<double, std::shared_ptr<Node>>>>;
using ClosedSet = std::unordered_set<std::shared_ptr<Node>>;
using CameFrom = std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<Node>>;
using GScore = std::unordered_map<std::shared_ptr<Node>, double>;
using FScore = std::unordered_map<std::shared_ptr<Node>, double>;

Path astar(std::shared_ptr<Node> const& start, std::shared_ptr<Node> const& goal,
           ufo::math::Vector3 velocity, bool reverse)
{
	std::vector<std::shared_ptr<Node>> goals;
	goals.push_back(goal);
	return astar(start, goals, velocity, reverse);
}

double minDistance(std::shared_ptr<Node> const& start,
                   std::vector<std::shared_ptr<Node>> const& goals)
{
	double min_squared_dist = std::numeric_limits<double>::infinity();
	for (std::shared_ptr<Node> const& goal : goals) {
		double squared_dist = start->squaredDistance(goal);
		if (squared_dist < min_squared_dist) {
			min_squared_dist = squared_dist;
		}
	}
	return sqrt(min_squared_dist);
}

double minDistance(std::shared_ptr<Node> const& start, ValueRtree const& goals)
{
	std::vector<Value> hits;
	goals.query(
	    bgi::nearest(Point(start->position[0], start->position[1], start->position[2]), 1),
	    std::back_inserter(hits));
	return start->distance(hits[0].second);
}

Path astar(std::shared_ptr<Node> const& start,
           std::vector<std::shared_ptr<Node>> const& goals, ufo::math::Vector3 velocity,
           bool reverse)
{
	std::unordered_set<std::shared_ptr<Node>> goals_set(goals.begin(), goals.end());

	// This temp container makes insertion into r-tree much faster
	std::vector<Value> temp;
	temp.reserve(goals_set.size());
	for (std::shared_ptr<Node> const& goal : goals_set) {
		temp.emplace_back(Point(goal->position[0], goal->position[1], goal->position[2]),
		                  goal);
	}
	ValueRtree goals_tree(temp.begin(), temp.end());

	OpenSet open;
	ClosedSet closed;
	CameFrom came_from;
	GScore g_score;
	FScore f_score;

	g_score[start] = 0.0;
	f_score[start] = minDistance(start, goals_tree);
	open.push(std::make_pair(f_score[start], start));

	std::shared_ptr<Node> current;
	while (!open.empty()) {
		current = open.top().second;
		open.pop();
		if (0 < closed.count(current)) {
			continue;
		}
		closed.insert(current);
		if (0 < goals_set.count(current)) {
			break;
		}
		for (std::shared_ptr<Node> const& neighbor : current->neighbors) {
			double tentative_g_score = g_score[current] + current->distance(neighbor);
			if (0 == g_score.count(neighbor) || tentative_g_score < g_score[neighbor]) {
				// if (start == current) {
				// 	tentative_g_score +=
				// 	    (neighbor->position - (current->position + velocity)).norm();
				// }
				came_from[neighbor] = current;
				g_score[neighbor] = tentative_g_score;
				f_score[neighbor] = tentative_g_score + minDistance(neighbor, goals_tree);
				open.push(std::make_pair(f_score[neighbor], neighbor));
			}
		}
	}

	if (0 == goals_set.count(current)) {
		// Did not find a path to any goal
		return std::make_pair(std::vector<std::shared_ptr<Node>>(),
		                      std::numeric_limits<double>::quiet_NaN());
	}

	// Reconstruct path
	std::vector<std::shared_ptr<Node>> path;
	path.push_back(current);
	while (0 != came_from.count(path.back())) {
		path.push_back(came_from[path.back()]);
	}

	if (reverse) {
		std::reverse(path.begin(), path.end());
	}

	return std::make_pair(path, g_score[current]);
}

std::vector<Path> astar(std::vector<std::shared_ptr<Node>> const& starts,
                        std::shared_ptr<Node> const& goal, bool reverse)
{
	std::vector<Path> paths;

	if (starts.empty()) {
		return paths;
	}

	OpenSet open;
	ClosedSet closed;
	CameFrom came_from;
	GScore g_score;
	FScore f_score;

	g_score[goal] = 0.0;
	f_score[goal] = goal->distance(starts[0]);
	open.push(std::make_pair(f_score[goal], goal));

	// Start from goal
	for (std::shared_ptr<Node> const& start : starts) {
		if (0 < closed.count(start)) {
			// Already found path to this
			continue;
		}

		{
			std::unordered_set<std::shared_ptr<Node>> open_nodes;
			while (!open.empty()) {
				open_nodes.insert(open.top().second);
				open.pop();
			}

			for (std::shared_ptr<Node> const& node : open_nodes) {
				f_score[node] = g_score[node] + node->distance(start);
				open.push(std::make_pair(f_score[node], node));
			}
		}

		while (!open.empty()) {
			std::shared_ptr<Node> current = open.top().second;
			open.pop();
			if (!closed.insert(current).second) {
				continue;
			}
			for (std::shared_ptr<Node> const& neighbor : current->neighbors) {
				double tentative_g_score = g_score[current] + current->distance(neighbor);
				if (0 == g_score.count(neighbor) || tentative_g_score < g_score[neighbor]) {
					came_from[neighbor] = current;
					g_score[neighbor] = tentative_g_score;
					f_score[neighbor] = tentative_g_score + neighbor->distance(start);
					open.push(std::make_pair(f_score[neighbor], neighbor));
				}
			}
			// We have it last since neighbors can be used by the other starts
			if (current == start) {
				break;
			}
		}
	}

	// Reconstruct paths
	for (std::shared_ptr<Node> const& start : starts) {
		std::vector<std::shared_ptr<Node>> path;
		path.push_back(start);
		while (0 != came_from.count(path.back())) {
			path.push_back(came_from[path.back()]);
		}

		if (!reverse) {
			std::reverse(path.begin(), path.end());
		}

		paths.push_back(std::make_pair(path, g_score[start]));
	}

	return paths;
}

Path biAstar(std::shared_ptr<Node> const& start, std::shared_ptr<Node> const& goal,
             bool reverse)
{
	OpenSet open_forward;
	OpenSet open_backward;

	ClosedSet closed_forward;
	ClosedSet closed_backward;

	CameFrom came_from_forward;
	CameFrom came_from_backward;

	GScore g_score_forward;
	GScore g_score_backward;

	FScore f_score_forward;
	FScore f_score_backward;

	g_score_forward[start] = 0.0;
	g_score_backward[goal] = 0.0;

	f_score_forward[start] = start->distance(goal);
	f_score_backward[goal] = goal->distance(start);

	open_forward.push(std::make_pair(f_score_forward[start], start));
	open_backward.push(std::make_pair(f_score_backward[goal], goal));

	std::shared_ptr<Node> current_forward;
	std::shared_ptr<Node> current_backward;

	std::shared_ptr<Node> connection;
	double min_length = std::numeric_limits<double>::infinity();

	bool forward = true;
	while (!open_forward.empty() && !open_backward.empty()) {
		OpenSet& open = forward ? open_forward : open_backward;
		ClosedSet& closed = forward ? closed_forward : closed_backward;
		CameFrom& came_from = forward ? came_from_forward : came_from_backward;
		GScore& g_score = forward ? g_score_forward : g_score_backward;
		FScore& f_score = forward ? f_score_forward : f_score_backward;
		std::shared_ptr<Node> const& target = forward ? goal : start;

		forward = !forward;

		std::shared_ptr<Node> current = open.top().second;
		open.pop();
		if (0 != closed.count(current)) {
			continue;
		}
		closed.insert(current);

		for (std::shared_ptr<Node> const& neighbor : current->neighbors) {
			double tentative_g_score = g_score[current] + current->distance(neighbor);
			if (0 == g_score.count(neighbor) || tentative_g_score < g_score[neighbor]) {
				came_from[neighbor] = current;
				g_score[neighbor] = tentative_g_score;
				f_score[neighbor] = tentative_g_score + neighbor->distance(target);
				open.push(std::make_pair(f_score[neighbor], neighbor));
			}
		}

		if (0 != closed_forward.count(current) && 0 != closed_backward.count(current)) {
			double length = g_score_forward[current] + g_score_backward[current];
			if (length < min_length) {
				connection = current;
				min_length = length;
			}
		}

		// Termination criteria
		// double possible_min_length = g_score_forward[open_forward.top().second] +
		//                              g_score_backward[open_backward.top().second];
		double possible_min_length = std::max(f_score_forward[open_forward.top().second],
		                                      f_score_backward[open_backward.top().second]);
		if (min_length <= possible_min_length) {
			break;
		}
	}

	// Reconstruct path
	std::vector<std::shared_ptr<Node>> path;
	path.push_back(connection);

	if (reverse) {
		// Forward
		while (0 != came_from_forward.count(path.back())) {
			path.push_back(came_from_forward[path.back()]);
		}

		std::reverse(path.begin(), path.end());

		// Backward
		while (0 != came_from_backward.count(path.back())) {
			path.push_back(came_from_backward[path.back()]);
		}
	} else {
		// Backward
		while (0 != came_from_backward.count(path.back())) {
			path.push_back(came_from_backward[path.back()]);
		}

		std::reverse(path.begin(), path.end());

		// Forward
		while (0 != came_from_forward.count(path.back())) {
			path.push_back(came_from_forward[path.back()]);
		}
	}

	return std::make_pair(path, min_length);
}
}  // namespace ufoexplorer