// UFO
#include <ufoexplorer/colors.h>
#include <ufoexplorer/explorer.h>
#include <ufoexplorer/graph_search.h>
#include <ufoexplorer/tsp_solver.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

// ROS
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

// Boost
#include <boost/dynamic_bitset.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <ctime>
#include <execution>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>

namespace ufoexplorer
{
Explorer::Explorer(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : gen_(rd_()),
      ac_("move_to"),
      tf_listener_(tf_buffer_),
      gain_worker_(&Explorer::gainWorkerThread, this),
      component_names_{
          "Map Update",      "Graph Updating", "Graph Expansion", "Clustering",
          "Distance Matrix", "Global Tour",    "Path Generation", "Update Gain",
          "Visualize",       "Whole"}
{
	map_sub_ = nh.subscribe("map_in", 100, &Explorer::mapCallback, this);

	// TODO: Subscribe to odometry/velocity
	odom_sub_ = nh.subscribe("odom", 100, &Explorer::odomCallback, this);

	// Start service
	run_service_ = nh_priv.advertiseService("run", &Explorer::runCallback, this);

	graph_pub_ = nh_priv.advertise<visualization_msgs::MarkerArray>("graph", 10, true);

	path_pub_ = nh_priv.advertise<nav_msgs::Path>("path", 10, true);
	global_tour_pub_ = nh_priv.advertise<nav_msgs::Path>("global_tour", 10, true);

	// Read parameters
	max_num_neighbors_ = nh_priv.param("graph/max_num_neighbors", 2);
	max_neighbor_dist_ = nh_priv.param("graph/max_neighbor_dist", 1.0);
	max_edge_length_ = nh_priv.param("graph/max_edge_length", 1.0);

	max_exact_tsp_ = nh_priv.param("graph/max_exact_tsp", 10);

	eps_ = nh_priv.param("dbscan/eps", 1);
	min_pts_ = nh_priv.param("dbscan/min_pts", 0);

	automatic_pruning_ = nh_priv.param("map/automatic_pruning", false);
	occupied_thres_ = nh_priv.param("map/occupied_thres", 0.5);
	free_thres_ = nh_priv.param("map/free_thres", 0.5);

	collision_depth_ = nh_priv.param("map/collision_depth", 0);
	gain_depth_ = nh_priv.param("map/gain_depth", 0);

	std::vector<double> bbx_min, bbx_max;
	if (!nh_priv.getParam("map/bbx_min", bbx_min)) {
		// TODO: Output something
		bbx_min.resize(3);
		for (double& i : bbx_min) {
			i = std::numeric_limits<double>::lowest();
		}
	}
	if (!nh_priv.getParam("map/bbx_max", bbx_max)) {
		// TODO: Output something
		bbx_max.resize(3);
		for (double& i : bbx_max) {
			i = std::numeric_limits<double>::max();
		}
	}

	total_volume_ =
	    (bbx_max[0] - bbx_min[0]) * (bbx_max[1] - bbx_min[1]) * (bbx_max[2] - bbx_min[2]);
	explorable_volume_ = nh_priv.param("explorable_volume", total_volume_);

	boundary_ = ufo::geometry::AABB(ufo::math::Vector3(bbx_min[0], bbx_min[1], bbx_min[2]),
	                                ufo::math::Vector3(bbx_max[0], bbx_max[1], bbx_max[2]));

	continue_outside_ = nh_priv.param("map/continue_outside", true);

	robot_.frame_id = nh_priv.param<std::string>("robot/frame_id", "base_link");
	robot_.radius = nh_priv.param("robot/radius", 0.4);
	robot_.height = nh_priv.param("robot/height", 0.2);

	sensor_.horizontal_fov = nh_priv.param("sensor/horizontal_fov", 80.0) * M_PI / 180.0;
	sensor_.vertical_fov = nh_priv.param("sensor/vertical_fov", 60.0) * M_PI / 180.0;
	sensor_.range_min = nh_priv.param("sensor/range_min", 0.0);
	sensor_.range_max = nh_priv.param("sensor/range_max", 7.0);

	min_gain_thres_ = nh_priv.param("min_gain_thres", 0.2);
	max_yaw_diff_ = nh_priv.param("max_yaw_diff", 2.0 * M_PI);

	running_ = nh_priv.param("start_immediately", true);
	exit_at_complete_ = nh_priv.param("exit_at_complete", false);

	verbose_ = nh_priv.param("verbose", false);

	if (nh_priv.param("save_computation_time_to_file", false)) {
		std::string file_path = ros::package::getPath("ufoexplorer") + "/data/";
		std::filesystem::create_directories(file_path);
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		std::ostringstream oss;
		oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
		std::string current_time = oss.str();
		std::string filename =
		    nh_priv.param<std::string>("filename", "ufoexplorer_output_" + current_time);
		if ("" == filename) {
			filename = "ufoexplorer_output_" + current_time;
		}
		output_file_.open(file_path + filename + ".tsv");
		output_file_ << "total (s)\texplored volume\texplored (%)\tunexplored (%)";
		for (std::string const& component_name : component_names_) {
			output_file_ << "\t" << component_name << " (ms)";
		}
		// output_file_ << "\tvelocity (m/s)\tyaw rate (rad/s)";
		output_file_ << "\n";
	}

	max_time_ = nh_priv.param("max_time", -1);
}

Explorer::~Explorer() { terminate(); }

void Explorer::terminate()
{
	gain_done_ = true;
	gain_cv_.notify_one();
	gain_worker_.join();

	if (output_file_.is_open()) {
		output_file_.close();
	}

	exit(0);
}

bool Explorer::runCallback(std_srvs::SetBool::Request& req,
                           std_srvs::SetBool::Response& res)
{
	running_ = req.data;
	res.success = true;
	return true;
}

void Explorer::odomCallback(nav_msgs::Odometry::ConstPtr const& msg)
{
	velocity_[0] = msg->twist.twist.linear.x;
	velocity_[1] = msg->twist.twist.linear.y;
	velocity_[2] = msg->twist.twist.linear.z;
	yaw_rate_ = msg->twist.twist.angular.z;
}

//
// Main functions
//

void Explorer::mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg)
{
	if (!running_) {
		// TODO: Fix
		// ufo::geometry::AABB aabb = getAABB(map_, msg);
		// explored_volume_ -= getExploredVolume(map_, aabb, boundary_, gain_depth_);
		// updateMap(map_, msg, automatic_pruning_, occupied_thres_, free_thres_);
		// explored_volume_ += getExploredVolume(map_, aabb, boundary_, gain_depth_);
		return;
	}

	if (!timing_.contains("Total")) {
		timing_.start("Total");
	}

	if (0 < max_time_ && max_time_ < timing_.getTotalWithCurrentSeconds("Total")) {
		ROS_WARN_THROTTLE(1.0, "Time limit exceeded, exploration stopped");
		if (exit_at_complete_) {
			terminate();
		}
		return;
	}

	// TODO: Fix
	// ufo::geometry::AABB aabb = getAABB(map_, msg);
	// explored_volume_ -= getExploredVolume(map_, aabb, boundary_, gain_depth_);

	timing_.start(component_names_[0]);
	auto o_aabb = updateMap(map_, msg, automatic_pruning_, occupied_thres_, free_thres_);
	timing_.stop(component_names_[0]);

	if (!o_aabb) {
		return;
	}
	ufo::geometry::AABB aabb = o_aabb.value();

	// explored_volume_ += getExploredVolume(map_, aabb, boundary_, gain_depth_);

	ufo::math::Vector3 diff = aabb.getMax() - aabb.getMin();
	double volume = diff[0] * diff[1] * diff[2];
	size_t samples = 2 * (max_num_neighbors_ / max_neighbor_dist_) * volume;
	if (0 == samples) {
		return;
	}

	timing_.start(component_names_[9]);

	// Update graph
	timing_.start(component_names_[1]);
	updateGraph(aabb);
	timing_.stop(component_names_[1]);

	// Update gain
	updateGain(aabb);

	// Extend graph
	timing_.start(component_names_[2]);
	extendGraph(aabb, samples);
	timing_.stop(component_names_[2]);

	if (graph_.empty()) {
		ROS_WARN_THROTTLE(1.0, "Graph is empty");
		return;
	}

	auto pos = getCurrentPosition(msg->header);
	if (!pos) {
		ROS_WARN_THROTTLE(1.0, "Cannot find current position of robot");
		return;
	}
	ufo::math::Vector3 current_position = pos.value();
	std::shared_ptr<Node> current_node = getNearestNode(current_position);
	if (nullptr == current_node) {
		ROS_WARN_THROTTLE(1.0,
		                  "Cannot find a node in the graph at the robots current position");
		timing_.start(component_names_[8]);
		visualize(msg->header);
		timing_.stop(component_names_[8]);
		return;
	}
	current_node->gain = 0.0;  // TODO: Hacky

	std::vector<std::shared_ptr<Node>> center_nodes;
	std::vector<std::shared_ptr<Node>> noise_cluster;
	std::vector<int> tour;
	std::vector<std::shared_ptr<Node>> path;
	std::vector<std::shared_ptr<Node>> nodes = getCandidateNodes();

	timing_.start(component_names_[3]);
	std::vector<std::vector<std::shared_ptr<Node>>> clusters =
	    getCandidateClusters(nodes, eps_, min_pts_, &noise_cluster);
	timing_.stop(component_names_[3]);

	if (clusters.empty()) {
		timing_.start(component_names_[8]);
		visualize(msg->header);
		timing_.stop(component_names_[8]);
		if (exploration_started_) {
			if (ac_.getState().isDone()) {
				ROS_WARN_THROTTLE(1.0, "Exploration complete");
				if (exit_at_complete_) {
					terminate();
				}
			} else {
				ROS_WARN_THROTTLE(1.0, "No information gain clusters found");
			}
		}
		return;
	}

	if (1 >= max_exact_tsp_) {
		timing_.start(component_names_[6]);
		path = shortenPath(getPath(current_node, nodes), current_position);
		timing_.stop(component_names_[6]);
	} else {
		std::vector<size_t> indices(clusters.size());
		std::generate(indices.begin(), indices.end(), [n = 0]() mutable { return n++; });
		center_nodes.resize(clusters.size() + 1);
		center_nodes[0] = current_node;
		std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
		              [this, &clusters, &center_nodes](auto index) {
			              center_nodes[index + 1] = getNodeClosestCentroid(clusters[index]);
		              });

		timing_.start(component_names_[4]);
		DistanceMatrix distance_matrix = getDistanceMatrix(center_nodes);
		timing_.stop(component_names_[4]);

		timing_.start(component_names_[5]);
		double cost;
		std::tie(tour, cost) = getTour(distance_matrix);
		timing_.stop(component_names_[5]);

		timing_.start(component_names_[6]);
		path = shortenPath(getPath(current_node, clusters, tour), current_position);
		timing_.stop(component_names_[6]);
	}

	if (path.empty()) {
		timing_.start(component_names_[8]);
		visualize(msg->header);
		timing_.stop(component_names_[8]);
		if (exploration_started_) {
			if (ac_.getState().isDone()) {
				ROS_WARN_THROTTLE(1.0, "Exploration complete");
				if (exit_at_complete_) {
					terminate();
				}
			} else {
				ROS_WARN_THROTTLE(1.0, "No path to goal candidates found");
			}
		}
		return;
	}

	exploration_started_ = true;

	publishPath(path, msg->header);

	timing_.stop(component_names_[9]);

	// Visualize
	timing_.start(component_names_[8]);
	visualize(msg->header, clusters, noise_cluster, center_nodes, tour);
	timing_.stop(component_names_[8]);

	if (verbose_) {
		printInfo();
	}

	writeComputationTimeToFile();
}

void Explorer::updateGraph(ufo::geometry::AABB region)
{
	// Remove edges
	std::vector<Value> nodes_in_collision;
	for (Value const& value : getValuesInBBX(region.getMin(), region.getMax())) {
		std::shared_ptr<Node> const& node = value.second;
		if (!inFreeSpace(map_, node->position, robot_, true, collision_depth_)) {
			nodes_in_collision.push_back(value);
		}
		std::vector<std::shared_ptr<Node>> neighbors(node->neighbors.begin(),
		                                             node->neighbors.end());
		for (std::shared_ptr<Node> const& neighbor : neighbors) {
			removeEdge(node, neighbor);
		}
	}

	// Remove nodes in collision
	graph_.remove(nodes_in_collision.begin(), nodes_in_collision.end());

	std::vector<std::tuple<size_t, std::shared_ptr<Node>, std::shared_ptr<Node>>> edges;

	// Add edges
	std::unordered_set<std::shared_ptr<Node>> processed;
	for (std::shared_ptr<Node> const& node :
	     getNodesInBBX(region.getMin(), region.getMax())) {
		processed.insert(node);
		for (std::shared_ptr<Node> const& neighbor :
		     getNodesInRadius(node->position, max_edge_length_)) {
			if (node == neighbor && 0 != processed.count(neighbor)) {
				continue;
			}
			edges.push_back(std::make_tuple(edges.size(), node, neighbor));
		}
	}

	boost::dynamic_bitset valid_edges(edges.size(), 0);
	// FIXME: Not valid to have std::vector<bool>
	// std::vector<bool> valid_edges(edges.size(), false);
	std::for_each(std::execution::par, edges.begin(), edges.end(),
	              [this, &valid_edges](auto&& edge) {
		              valid_edges[std::get<0>(edge)] = isCollisionFreeLine(
		                  map_, std::get<1>(edge)->position, std::get<2>(edge)->position,
		                  robot_, true, collision_depth_);
	              });

	for (auto const& [index, node, neighbor] : edges) {
		if (valid_edges[index]) {
			addEdge(node, neighbor);
		}
	}
}

void Explorer::extendGraph(ufo::geometry::AABB region, size_t samples)
{
	ufo::math::Vector3 min = region.getMin();
	ufo::math::Vector3 max = region.getMax();
	ufo::geometry::Point boundary_min = boundary_.getMin();
	ufo::geometry::Point boundary_max = boundary_.getMax();
	for (int i : {0, 1, 2}) {
		min[i] = std::max(min[i], boundary_min[i]);
		max[i] = std::min(max[i], boundary_max[i]);
	}

	std::vector<std::shared_ptr<Node>> new_nodes;
	new_nodes.reserve(samples);

	for (size_t i = 0; i < samples; ++i) {
		// Sample new position
		ufo::math::Vector3 position = samplePosition(min, max);

		if (!graph_.empty()) {
			// Get closest node
			std::shared_ptr<Node> closest_node = getNearestNode(position);

			// Move position closer to closest_node
			position = restrictDistance(position, closest_node->position);
		}

		if (!inFreeSpace(map_, position, robot_, true, collision_depth_)) {
			continue;
		}

		// Get all nodes close by
		std::vector<std::shared_ptr<Node>> neighbors =
		    getNodesInRadius(position, max_neighbor_dist_);

		if (neighbors.size() > max_num_neighbors_) {
			// Too many nodes in this region
			continue;
		}

		// Add a new node
		std::shared_ptr<Node> new_node = std::make_shared<Node>();
		new_node->position = position;

		// Get all potential neighbors
		neighbors = getNodesInRadius(position, max_edge_length_);

		// Add edges
		for (std::shared_ptr<Node> const& neighbor : neighbors) {
			if (isCollisionFreeLine(map_, new_node->position, neighbor->position, robot_, true,
			                        collision_depth_)) {
				addEdge(new_node, neighbor);
			}
		}

		if (new_node->neighbors.empty() && !graph_.empty()) {
			// Not connected to anything
			continue;
		}

		// Add node to graph
		addNode(new_node);

		new_nodes.push_back(new_node);
	}

	// Calculate gain for new nodes
	std::for_each(std::execution::par_unseq, new_nodes.begin(), new_nodes.end(),
	              [this](std::shared_ptr<Node> const& node) {
		              std::tie(node->gain, node->yaw) =
		                  getGain(map_, node->position, boundary_, sensor_, continue_outside_,
		                          gain_depth_);
	              });
}

std::vector<std::shared_ptr<Node>> Explorer::getCandidateNodes() const
{
	std::vector<std::shared_ptr<Node>> nodes;
	nodes.reserve(graph_.size());
	for (auto& [p, node] : graph_) {
		if (min_gain_thres_ < node->gain) {
			nodes.push_back(node);
		}
	}
	return nodes;
}

std::vector<std::vector<std::shared_ptr<Node>>> Explorer::getCandidateClusters(
    std::vector<std::shared_ptr<Node>> const& nodes, int eps, int min_pts,
    std::vector<std::shared_ptr<Node>>* noise_cluster) const
{
	if (0 == min_pts) {
		if (nodes.empty()) {
			return std::vector<std::vector<std::shared_ptr<Node>>>();
		}
		return std::vector<std::vector<std::shared_ptr<Node>>>{nodes};
	}

	return dbscan(nodes, eps, min_pts, noise_cluster);
}

std::optional<ufo::math::Vector3> Explorer::getCurrentPosition(
    std_msgs::Header const& map_header) const
{
	if (tf_buffer_.canTransform(map_header.frame_id, robot_.frame_id, map_header.stamp,
	                            ros::Duration(0.1))) {
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = robot_.frame_id;
		pose.header.stamp = map_header.stamp;
		pose.pose.orientation.w = 1.0;
		pose = tf_buffer_.transform(pose, map_header.frame_id);
		return ufomap_ros::rosToUfo(pose.pose.position);
	}
	return std::nullopt;
}

std::shared_ptr<Node> Explorer::getCurrentNode(std_msgs::Header const& map_header) const
{
	auto position = getCurrentPosition(map_header);
	if (position) {
		return getNearestNode(position.value());
	}
	return nullptr;
}

std::shared_ptr<Node> Explorer::getNodeClosestCentroid(
    std::vector<std::shared_ptr<Node>> const& nodes) const
{
	ufo::math::Vector3 centroid = getCentroid(nodes);
	return *std::min_element(std::execution::par_unseq, nodes.begin(), nodes.end(),
	                         [&centroid](auto const& node_1, auto const& node_2) {
		                         return (centroid - node_1->position).squaredNorm() <
		                                (centroid - node_2->position).squaredNorm();
	                         });
}

ufo::math::Vector3 Explorer::getCentroid(
    std::vector<std::shared_ptr<Node>> const& nodes) const
{
	ufo::math::Vector3 centroid;
	for (std::shared_ptr<Node> const& node : nodes) {
		centroid += node->position;
	}
	return centroid / nodes.size();
}

DistanceMatrix Explorer::getDistanceMatrix(
    std::vector<std::shared_ptr<Node>> const& nodes) const
{
	DistanceMatrix distance_matrix(nodes.size(), std::vector<double>(nodes.size(), 0.0));

	std::vector<size_t> indices(distance_matrix.size() - 1);
	std::generate(indices.begin(), indices.end(), [n = 1]() mutable { return n++; });

	std::for_each(
	    std::execution::par_unseq, indices.begin(), indices.end(),
	    [&nodes, &distance_matrix](auto index) {
		    size_t j = 0;
		    for (auto [path, dist] : astar(
		             std::vector<std::shared_ptr<Node>>(nodes.begin(), nodes.begin() + index),
		             nodes[index], true)) {
			    distance_matrix[index][j] = dist;
			    distance_matrix[j][index] = dist;
			    ++j;
		    }
	    });
	return distance_matrix;
}

Tour Explorer::getTour(std::vector<std::vector<double>> const& distance_matrix) const
{
	if (distance_matrix.size() <= max_exact_tsp_) {
		return TSPBruteForce(distance_matrix, 0);
	} else {
		return TSPNN(distance_matrix, 0);
	}
}

std::vector<std::shared_ptr<Node>> Explorer::getPath(
    std::shared_ptr<Node> const& current_position,
    std::vector<std::shared_ptr<Node>> const& nodes) const
{
	if ("exponential_penalty" == heuristic_) {
		// Exponential
		std::vector<std::shared_ptr<Node>> best_path;
		double best_score = std::numeric_limits<double>::lowest();
		for (auto [path, c] : astar(nodes, current_position, false)) {
			if (path.empty()) {
				continue;
			}
			double cost = 0.0;
			double score = min_gain_thres_ < path[0]->gain ? path[0]->gain : 0.0;
			for (size_t i = 1; i < path.size(); ++i) {
				double gain = min_gain_thres_ < path[i]->gain ? path[i]->gain : 0.0;
				cost += path[i]->distance(path[i - 1]);
				score += gain * std::exp(-0.5 * cost);
			}
			if (best_score < score) {
				best_score = score;
				best_path = path;
			}
		}
		return best_path;
	} else if ("global_normalization" == heuristic_) {
		// Global normalization
		std::vector<std::shared_ptr<Node>> best_path;
		double best_score = std::numeric_limits<double>::lowest();
		for (auto [path, cost] : astar(nodes, current_position, false)) {
			if (path.empty()) {
				continue;
			}
			double gain = 0.0;
			for (auto node : path) {
				gain += min_gain_thres_ < node->gain ? node->gain : 0.0;
			}
			double score = gain / cost;
			if (best_score < score) {
				best_score = score;
				best_path = path;
			}
		}
		return best_path;
	}

	// Closest node with information gain
	return astar(current_position, nodes, velocity_, true).first;
}

std::vector<std::shared_ptr<Node>> Explorer::getPath(
    std::shared_ptr<Node> const& current_position,
    std::vector<std::vector<std::shared_ptr<Node>>> const& clusters,
    std::vector<int> const& tour) const
{
	return getPath(current_position, clusters[tour[1] - 1]);

	// std::vector<std::shared_ptr<Node>> best_path;
	// double best_cost = std::numeric_limits<double>::max();

	// std::vector<std::shared_ptr<Node>> const& nodes = clusters[tour[1] - 1];

	// for (auto [path, dist] : astar(nodes, current_position, true)) {
	// 	// std::shared_ptr<Node> const& goal = path.front();

	// 	double cost = dist;
	// 	if (cost < best_cost) {
	// 		best_cost = cost;
	// 		std::swap(best_path, path);
	// 	}
	// }

	// return best_path;
}

std::vector<std::shared_ptr<Node>> Explorer::shortenPath(
    std::vector<std::shared_ptr<Node>> const& path,
    ufo::math::Vector3 const& current_position) const
{
	if (path.empty()) {
		return path;
	}

	std::vector<std::shared_ptr<Node>> short_path;

	size_t index = 0;
	for (; index < path.size() - 1; ++index) {
		if (!isCollisionFreeLine(map_, current_position, path[index + 1]->position, robot_,
		                         true, collision_depth_)) {
			short_path.push_back(path[index]);
			break;
		}
	}

	if (short_path.empty()) {
		return {path.back()};
	}

	short_path.reserve(path.size() - index);

	for (++index; index < path.size() - 1; ++index) {
		if (!isCollisionFreeLine(map_, short_path.back()->position, path[index + 1]->position,
		                         robot_, true, collision_depth_)) {
			short_path.push_back(path[index]);
		}
	}

	short_path.push_back(path.back());

	return short_path;
}

std::vector<std::shared_ptr<Node>> Explorer::shortenPath(
    std::vector<std::shared_ptr<Node>> const& path) const
{
	if (2 > path.size()) {
		return path;
	}

	std::vector<std::shared_ptr<Node>> short_path;
	short_path.reserve(path.size() - 1);
	// if (!path.empty()) {
	// short_path.push_back(path.front());
	short_path.push_back(path[1]);
	for (size_t i = 2; i < path.size() - 1; ++i) {
		if (!isCollisionFreeLine(map_, short_path.back()->position, path[i + 1]->position,
		                         robot_, true, collision_depth_)) {
			short_path.push_back(path[i]);
		}
	}
	// if (2 <= path.size()) {
	if (3 <= path.size()) {
		short_path.push_back(path.back());
	}
	// }
	return short_path;
}

void Explorer::publishPath(std::vector<std::shared_ptr<Node>> const& path,
                           std_msgs::Header const& map_header)
{
	collision_avoidance::PathControlGoal goal;
	goal.path.header = map_header;

	goal.look_forward.resize(path.size(), 0);
	// for (size_t i = 0; i < path.size(); ++i) {
	// 	if (min_gain_thres_ > path[i]->gain) {
	// 		// Very low gain, set yaw to next node
	// 		goal.look_forward[i] = 1;
	// 	}
	// }

	std::vector<size_t> indices(path.size());
	std::generate(indices.begin(), indices.end(), [n = 0]() mutable { return n++; });

	goal.path.poses.resize(path.size());
	std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
	              [this, &path, &goal](size_t index) {
		              std::shared_ptr<Node> const& node = path[index];
		              geometry_msgs::PoseStamped pose;
		              pose.header = goal.path.header;
		              pose.pose.position = ufomap_ros::ufoToRos(node->position);
		              tf2::Quaternion q;
		              // if (node->distance(path.back()) > max_edge_length_) {
		              // SensorModel sensor = sensor_;
		              // // TODO: Do not hardcode these
		              // sensor.range_min = 0.5;
		              // sensor.range_max = 7.0;
		              // auto [gain, yaw] = getGain(map_, node->position, boundary_, sensor,
		              //                            continue_outside_, gain_depth_);
		              q.setRPY(0.0, 0.0, path.back()->yaw);
		              // } else {
		              //   q.setRPY(0.0, 0.0, path.back()->yaw);
		              // }
		              pose.pose.orientation = tf2::toMsg(q);
		              goal.path.poses[index] = pose;
	              });

	// for (std::shared_ptr<Node> const& node : path) {
	// 	pose.pose.position = ufomap_ros::ufoToRos(node->position);
	// 	tf2::Quaternion q;
	// 	if (node->squaredDistance(path.back()) > max_yaw_dist) {
	// 		auto [gain, yaw] = getGain(map_, node->position, boundary_, sensor_,
	// 		                           continue_outside_, gain_depth_);
	// 		q.setRPY(0.0, 0.0, yaw);
	// 	} else {
	// 		q.setRPY(0.0, 0.0, path.back()->yaw);
	// 	}
	// 	pose.pose.orientation = tf2::toMsg(q);
	// 	goal.path.poses.push_back(pose);
	// }

	// if (1 < goal.path.poses.size()) {
	// 	// Remove first since it can be in the wrong direction making the robot "wiggle".
	// 	// Not super important, works without this.
	// 	goal.path.poses.erase(goal.path.poses.begin());
	// 	goal.look_forward.erase(goal.look_forward.begin());
	// }

	goal.go_to_every_target = false;
	goal.do_avoidance = true;
	goal.max_times_backwards = 10;
	ac_.sendGoal(goal);
}

void Explorer::visualize(std_msgs::Header const& map_header,
                         std::vector<std::vector<std::shared_ptr<Node>>> const& clusters,
                         std::vector<std::shared_ptr<Node>> const& noise_cluster,
                         std::vector<std::shared_ptr<Node>> const& center_nodes,
                         std::vector<int> const& tour) const
{
	if (0 == graph_pub_.getNumSubscribers() && !graph_pub_.isLatched()) {
		return;
	}

	visualization_msgs::MarkerArray markers;

	// Clear all existing markers
	visualization_msgs::Marker clear_marker;
	clear_marker.header = map_header;
	clear_marker.action = visualization_msgs::Marker::DELETEALL;
	markers.markers.push_back(clear_marker);

	// Add new markers
	visualization_msgs::Marker vertices_marker;
	vertices_marker.header = map_header;
	vertices_marker.id = 0;
	vertices_marker.ns = "vertices";
	vertices_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	vertices_marker.action = visualization_msgs::Marker::ADD;
	vertices_marker.pose.orientation.w = 1.0;
	vertices_marker.scale.x = 0.3;
	vertices_marker.scale.y = 0.3;
	vertices_marker.scale.z = 0.3;
	vertices_marker.color.a = 1.0;
	vertices_marker.color.r = 1.0;
	vertices_marker.color.g = 0.498039215686;
	vertices_marker.color.b = 0.0549019607843;
	vertices_marker.lifetime = ros::Duration(0.0);  // Forever
	vertices_marker.frame_locked = false;

	visualization_msgs::Marker edges_marker;
	edges_marker.header = map_header;
	edges_marker.id = 1;
	edges_marker.ns = "edges";
	edges_marker.type = visualization_msgs::Marker::LINE_LIST;
	edges_marker.action = visualization_msgs::Marker::ADD;
	edges_marker.pose.orientation.w = 1.0;  // To remove RViz warning
	edges_marker.scale.x = 0.05;
	edges_marker.color.a = 1.0;
	edges_marker.color.r = 0.121568627451;
	edges_marker.color.g = 0.466666666667;
	edges_marker.color.b = 0.705882352941;       // 1.0 - node->gain(map_);
	edges_marker.lifetime = ros::Duration(0.0);  // Forever
	edges_marker.frame_locked = false;

	size_t id = 2;
	for (auto& [p, node] : graph_) {
		geometry_msgs::Point position;
		position.x = node->position[0];
		position.y = node->position[1];
		position.z = node->position[2];

		vertices_marker.points.push_back(position);

		for (auto& neighbor : node->neighbors) {
			if (node->position[0] > neighbor->position[0]) {
				// To not have duplicated edges
				continue;
			}

			geometry_msgs::Point n_position;
			n_position.x = neighbor->position[0];
			n_position.y = neighbor->position[1];
			n_position.z = neighbor->position[2];

			edges_marker.points.push_back(position);
			edges_marker.points.push_back(n_position);
		}

		if (min_gain_thres_ < node->gain) {
			visualization_msgs::Marker gain_marker;
			gain_marker.header = map_header;
			gain_marker.id = id++;
			gain_marker.ns = "gain";
			gain_marker.type = visualization_msgs::Marker::SPHERE;
			gain_marker.action = visualization_msgs::Marker::ADD;
			gain_marker.pose.position = position;
			gain_marker.pose.orientation.w = 1.0;
			gain_marker.scale.x = node->gain;
			gain_marker.scale.y = node->gain;
			gain_marker.scale.z = node->gain;
			gain_marker.color.a = node->gain;
			gain_marker.color.r = 0.0;
			gain_marker.color.g = 0.0;
			gain_marker.color.b = 1.0;
			gain_marker.lifetime = ros::Duration(0.0);  // Forever
			gain_marker.frame_locked = false;
			markers.markers.push_back(gain_marker);

			visualization_msgs::Marker yaw_marker;
			yaw_marker.header = map_header;
			yaw_marker.id = id++;
			yaw_marker.ns = "yaw";
			yaw_marker.type = visualization_msgs::Marker::ARROW;
			yaw_marker.action = visualization_msgs::Marker::ADD;
			yaw_marker.pose.position = position;
			tf2::Quaternion yaw_q;
			yaw_q.setRPY(0, 0, node->yaw);
			yaw_marker.pose.orientation = tf2::toMsg(yaw_q);
			yaw_marker.scale.x = 0.4;
			yaw_marker.scale.y = 0.05;
			yaw_marker.scale.z = 0.05;
			yaw_marker.color.a = 1.0;
			yaw_marker.color.r = 1.0;
			yaw_marker.color.g = 1.0;
			yaw_marker.color.b = 0.0;                  // 1.0 - node->gain(map_);
			yaw_marker.lifetime = ros::Duration(0.0);  // Forever
			yaw_marker.frame_locked = false;
			markers.markers.push_back(yaw_marker);
		}
	}

	// Visualize clusters
	visualization_msgs::Marker cluster_marker;
	cluster_marker.header = map_header;
	cluster_marker.id = id++;
	cluster_marker.ns = "clusters";
	cluster_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	cluster_marker.action = visualization_msgs::Marker::ADD;
	cluster_marker.pose.orientation.w = 1.0;
	cluster_marker.scale.x = 0.3;
	cluster_marker.scale.y = 0.3;
	cluster_marker.scale.z = 0.3;
	cluster_marker.lifetime = ros::Duration(0.0);  // Forever
	cluster_marker.frame_locked = false;
	for (size_t label = 0; label < clusters.size(); ++label) {
		std_msgs::ColorRGBA color;
		color.a = 1.0;
		int color_index =
		    round(((label + 1.0) / clusters.size()) * (turbo_srgb_floats.size() - 1));
		color.r = turbo_srgb_floats[color_index][0];
		color.g = turbo_srgb_floats[color_index][1];
		color.b = turbo_srgb_floats[color_index][2];

		for (std::shared_ptr<Node> const& node : clusters[label]) {
			geometry_msgs::Point position;
			position.x = node->position[0];
			position.y = node->position[1];
			position.z = node->position[2];

			cluster_marker.points.push_back(position);
			cluster_marker.colors.push_back(color);
		}
	}

	visualization_msgs::Marker noise_cluster_marker;
	noise_cluster_marker.header = map_header;
	noise_cluster_marker.id = id++;
	noise_cluster_marker.ns = "noise_clusters";
	noise_cluster_marker.type = visualization_msgs::Marker::CUBE_LIST;
	noise_cluster_marker.action = visualization_msgs::Marker::ADD;
	noise_cluster_marker.pose.orientation.w = 1.0;
	noise_cluster_marker.scale.x = 0.3;
	noise_cluster_marker.scale.y = 0.3;
	noise_cluster_marker.scale.z = 0.3;
	noise_cluster_marker.lifetime = ros::Duration(0.0);  // Forever
	noise_cluster_marker.frame_locked = false;
	noise_cluster_marker.color.a = 1.0;
	// Pink
	noise_cluster_marker.color.r = 227 / 255.0;
	noise_cluster_marker.color.g = 119 / 255.0;
	noise_cluster_marker.color.b = 194 / 255.0;
	for (std::shared_ptr<Node> const& node : noise_cluster) {
		geometry_msgs::Point position;
		position.x = node->position[0];
		position.y = node->position[1];
		position.z = node->position[2];
		noise_cluster_marker.points.push_back(position);
	}

	if (!vertices_marker.points.empty()) {
		markers.markers.push_back(vertices_marker);
	}
	if (!edges_marker.points.empty()) {
		markers.markers.push_back(edges_marker);
	}
	if (!cluster_marker.points.empty()) {
		markers.markers.push_back(cluster_marker);
	}
	if (!noise_cluster_marker.points.empty()) {
		markers.markers.push_back(noise_cluster_marker);
	}
	graph_pub_.publish(markers);

	// Publish tour
	nav_msgs::Path global_tour;
	global_tour.header = map_header;
	geometry_msgs::PoseStamped tour_pose;
	tour_pose.header = global_tour.header;
	for (int i : tour) {
		tf2::Quaternion q;
		tour_pose.pose.position = ufomap_ros::ufoToRos(center_nodes[i]->position);
		q.setRPY(0.0, 0.0, center_nodes[i]->yaw);
		tour_pose.pose.orientation = tf2::toMsg(q);
		global_tour.poses.push_back(tour_pose);
	}
	global_tour_pub_.publish(global_tour);
}

void Explorer::visualize(std_msgs::Header const& map_header) const
{
	visualize(map_header, std::vector<std::vector<std::shared_ptr<Node>>>(),
	          std::vector<std::shared_ptr<Node>>(), std::vector<std::shared_ptr<Node>>(),
	          std::vector<int>());
}

void Explorer::printInfo() const
{
	printf("Timings\n");
	printf("\tTotal    %s%4.0f s%s\n", BOLDBLUE,
	       timing_.getTotalWithCurrentSeconds("Total"), RESET);
	printf("\tExplored %s%5.2f%%%s (%.2f / %.2f m^3)\n", BOLDRED,
	       100.0 * (explored_volume_ / explorable_volume_), RESET, explored_volume_,
	       explorable_volume_);
	printf("\tNumber of nodes %s%lu%s, Candidate nodes %s%lu%s\n", BOLDGREEN, graph_.size(),
	       RESET, BOLDGREEN, getCandidateNodes().size(), RESET);
	printf(
	    "\t    Component               %sCurrent    %sMean     %sStD      %sMin      "
	    "%sMax%s\n",
	    BLUE, MAGENTA, CYAN, GREEN, RED, RESET);

	for (std::string const& component_name : component_names_) {
		if (timing_.contains(component_name)) {
			printf("\t%-20s (ms)   %s%6.2f   %s%6.2f   %s%6.2f   %s%6.2f   %s%6.2f%s\n",
			       component_name.c_str(), BLUE, timing_.getLastMilliseconds(component_name),
			       MAGENTA, timing_.getMeanMilliseconds(component_name), CYAN,
			       timing_.getStdMilliseconds(component_name), GREEN,
			       timing_.getMinMilliseconds(component_name), RED,
			       timing_.getMaxMilliseconds(component_name), RESET);
		}
	}
}

void Explorer::writeComputationTimeToFile()
{
	if (output_file_.is_open()) {
		output_file_ << timing_.getTotalWithCurrentSeconds("Total");
		output_file_ << "\t" << explored_volume_;
		output_file_ << "\t" << 100.0 * (explored_volume_ / explorable_volume_);
		output_file_ << "\t" << 100.0 * (1.0 - (explored_volume_ / explorable_volume_));
		for (std::string const& component_name : component_names_) {
			if (timing_.contains(component_name)) {
				output_file_ << "\t" << timing_.getLastMilliseconds(component_name);
			} else {
				output_file_ << "\t" << NAN;
			}
		}
		// output_file_ << "\t" << velocity_.norm() << "\t" << yaw_rate_;
		output_file_ << "\n";
	}
}

//
// Helper functions
//

double Explorer::dynamicPenalty(ufo::math::Vector3 const& from,
                                ufo::math::Vector3 const& to,
                                ufo::math::Vector3 const& vel) const
{
	ufo::math::Vector3 temp = to - from;
	return acos(ufo::math::Vector3::dot(temp, vel) / (temp.norm() * vel.norm()));
}

// void Explorer::graphWorkerThread() {

// }

// void Explorer::planningWorkerThread() {

// }

void Explorer::gainWorkerThread()
{
	while (!gain_done_) {
		std::unique_lock<std::mutex> lk(gain_mutex_);
		gain_cv_.wait(lk, [this] { return !gain_queue_.empty() || gain_done_; });

		if (gain_done_) {
			break;
		}

		timing_.start(component_names_[7]);
		std::unordered_set<std::shared_ptr<Node>> local_queue(gain_queue_.bucket_count());
		std::swap(local_queue, gain_queue_);

		lk.unlock();

		std::for_each(std::execution::par_unseq, local_queue.begin(), local_queue.end(),
		              [this](std::shared_ptr<Node> const& node) {
			              if (min_gain_thres_ < node->gain) {
				              std::tie(node->gain, node->yaw) =
				                  getGain(map_, node->position, boundary_, sensor_,
				                          continue_outside_, gain_depth_);
			              }
		              });
		timing_.stop(component_names_[7]);
	}
}

void Explorer::updateGain(ufo::geometry::AABB region)
{
	// Increase half size to cover all nodes which might see into region
	region.half_size += sensor_.range_max;  // TODO: Update, take into account vertical FoV

	std::vector<std::shared_ptr<Node>> nodes =
	    getNodesInBBX(region.getMin(), region.getMax());

	{
		std::lock_guard<std::mutex> lk(gain_mutex_);
		for (std::shared_ptr<Node> const& node : nodes) {
			if (min_gain_thres_ < node->gain) {
				gain_queue_.insert(node);
			}
		}
	}

	gain_cv_.notify_one();
}

ufo::math::Vector3 Explorer::restrictDistance(ufo::math::Vector3 const& position,
                                              ufo::math::Vector3 const& closest) const
{
	ufo::math::Vector3 direction = position - closest;
	if (direction.norm() > max_edge_length_) {
		direction = direction.normalized() * max_edge_length_;
		return closest + direction;
	}
	return position;
}

ufo::math::Vector3 Explorer::samplePosition(ufo::math::Vector3 const& min,
                                            ufo::math::Vector3 const& max)
{
	ufo::math::Vector3 sample;
	for (int i : {0, 1, 2}) {
		real_dist_.param(std::uniform_real_distribution<double>::param_type(min[i], max[i]));
		sample[i] = real_dist_(gen_);
	}
	return sample;
}

void Explorer::addEdge(std::shared_ptr<Node> const& node_1,
                       std::shared_ptr<Node> const& node_2)
{
	node_1->neighbors.insert(node_2);
	node_2->neighbors.insert(node_1);
}

void Explorer::removeEdge(std::shared_ptr<Node> const& node_1,
                          std::shared_ptr<Node> const& node_2)
{
	node_1->neighbors.erase(node_2);
	node_2->neighbors.erase(node_1);
}

void Explorer::addNode(std::shared_ptr<Node> const& node)
{
	graph_.insert(std::make_pair(
	    Point(node->position[0], node->position[1], node->position[2]), node));
}

void Explorer::deleteNode(std::shared_ptr<Node> const& node)
{
	if (!node->neighbors.empty()) {
		// TODO: Error
	}

	graph_.remove(std::make_pair(
	    Point(node->position[0], node->position[1], node->position[2]), node));
}

std::shared_ptr<Node> Explorer::getNearestNode(ufo::math::Vector3 const& position) const
{
	return getNearestValue(position).second;
}

std::vector<std::shared_ptr<Node>> Explorer::getKNearestNodes(
    ufo::math::Vector3 const& position, int k) const
{
	std::vector<std::shared_ptr<Node>> k_nearest;
	for (Value const& v : getKNearestValues(position, k)) {
		k_nearest.push_back(v.second);
	}
	return k_nearest;
}

std::vector<std::shared_ptr<Node>> Explorer::getNodesInRadius(
    ufo::math::Vector3 const& position, double radius) const
{
	std::vector<std::shared_ptr<Node>> return_nodes;
	for (Value const& v : getValuesInRadius(position, radius)) {
		return_nodes.push_back(v.second);
	}
	return return_nodes;
}

std::vector<std::shared_ptr<Node>> Explorer::getNodesInBBX(
    ufo::math::Vector3 const& min, ufo::math::Vector3 const& max) const
{
	std::vector<std::shared_ptr<Node>> return_nodes;
	for (Value const& v : getValuesInBBX(min, max)) {
		return_nodes.push_back(v.second);
	}
	return return_nodes;
}

Value Explorer::getNearestValue(ufo::math::Vector3 const& position) const
{
	std::vector<Value> hits;
	graph_.query(bgi::nearest(Point(position[0], position[1], position[2]), 1),
	             std::back_inserter(hits));

	assert(hits.size() == 1);
	return hits[0];
}

std::vector<Value> Explorer::getKNearestValues(ufo::math::Vector3 const& position,
                                               int k) const
{
	std::vector<Value> hits;
	graph_.query(bgi::nearest(Point(position[0], position[1], position[2]), k),
	             std::back_inserter(hits));
	return hits;
}

std::vector<Value> Explorer::getValuesInRadius(ufo::math::Vector3 const& position,
                                               double radius) const
{
	std::vector<Value> hits;
	Point sought(position[0], position[1], position[2]);
	graph_.query(bgi::satisfies([&](Value const& v) {
		             return bg::distance(v.first, sought) < radius;
	             }),
	             std::back_inserter(hits));
	return hits;
}

std::vector<Value> Explorer::getValuesInBBX(ufo::math::Vector3 const& min,
                                            ufo::math::Vector3 const& max) const
{
	Box query_box(Point(min[0], min[1], min[2]), Point(max[0], max[1], max[2]));
	std::vector<Value> hits;
	graph_.query(bgi::intersects(query_box), std::back_inserter(hits));
	return hits;
}

//
// Clustering
//

inline double getAbsoluteDiff2Angles(const double x, const double y, const double c)
{
	// c can be PI (for radians) or 180.0 (for degrees);
	return c - fabs(fmod(fabs(x - y), 2 * c) - c);
}

std::vector<std::shared_ptr<Node>> Explorer::dbscanRangeQuery(
    std::shared_ptr<Node> const& node, double yaw, int eps) const
{
	std::queue<std::pair<std::shared_ptr<Node>, int>> open;
	std::unordered_set<std::shared_ptr<Node>> closed;

	open.push(std::make_pair(node, 0));
	closed.insert(node);

	std::vector<std::shared_ptr<Node>> neighbors;
	while (!open.empty()) {
		auto [current_node, distance] = open.front();
		open.pop();

		if (min_gain_thres_ < current_node->gain &&
		    max_yaw_diff_ >= getAbsoluteDiff2Angles(yaw, current_node->yaw, M_PI)) {
			neighbors.push_back(current_node);
		}

		if (distance < eps) {
			for (std::shared_ptr<Node> const& neighbor : current_node->neighbors) {
				if (closed.insert(neighbor).second) {
					open.push(std::make_pair(neighbor, distance + 1));
				}
			}
		}
	}

	return neighbors;
}

std::vector<std::vector<std::shared_ptr<Node>>> Explorer::dbscan(
    std::vector<std::shared_ptr<Node>> const& nodes, int eps, int min_pts,
    std::vector<std::shared_ptr<Node>>* noise_cluster) const
{
	std::unordered_map<std::shared_ptr<Node>, int> node_label;

	int idx_current_cluster = UNDEFINED_CLUSTER;  // Cluster counter
	for (std::shared_ptr<Node> const& node : nodes) {
		if (UNDEFINED_CLUSTER != node_label.count(node)) {
			continue;  // Previously processed in inner loop
		}
		double yaw = node->yaw;
		// Find neighbors
		std::vector<std::shared_ptr<Node>> neighbors = dbscanRangeQuery(node, yaw, eps);
		if (neighbors.size() < min_pts) {    // Density check
			node_label[node] = NOISE_CLUSTER;  // Label as noise
			continue;
		}
		++idx_current_cluster;                   // Next cluster label
		node_label[node] = idx_current_cluster;  // Label initial point
		for (size_t i = 0; i < neighbors.size(); ++i) {
			std::shared_ptr<Node> const& neighbor = neighbors[i];
			int& neighbor_label = node_label[neighbor];
			if (NOISE_CLUSTER == neighbor_label) {
				// Change noise to border point
				neighbor_label = idx_current_cluster;
			}
			if (UNDEFINED_CLUSTER != neighbor_label) {
				// Previously processed (e.g., border point)
				continue;
			}
			neighbor_label = idx_current_cluster;  // Label neighbor
			std::vector<std::shared_ptr<Node>> new_neighbors =
			    dbscanRangeQuery(neighbor, yaw, eps);
			if (new_neighbors.size() >= min_pts) {
				// Add new neighbors
				neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
			}
		}
	}

	std::vector<std::vector<std::shared_ptr<Node>>> clusters(idx_current_cluster);
	for (auto& [node, label] : node_label) {
		if (nullptr != noise_cluster && NOISE_CLUSTER == label) {
			noise_cluster->push_back(node);
		} else if (NOISE_CLUSTER != label && UNDEFINED_CLUSTER != label) {
			clusters[label - 1].push_back(node);
		}
	}
	return clusters;
}
}  // namespace ufoexplorer