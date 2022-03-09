#ifndef UFOEXPLORER_EXPLORER_H
#define UFOEXPLORER_EXPLORER_H

// UFO
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufo/math/vector3.h>

// UFOExplorer
#include <ufoexplorer/map.h>  // Not really needed here
#include <ufoexplorer/model/robot_model.h>
#include <ufoexplorer/model/sensor_model.h>
#include <ufoexplorer/node.h>
#include <ufoexplorer/timing.h>
#include <ufoexplorer/tsp_solver.h>  // Not really needed here

// ROS
#include <actionlib/client/simple_action_client.h>
#include <collision_avoidance/PathControlAction.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ufomap_msgs/UFOMapStamped.h>

// Boost
#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

// STL
#include <chrono>
#include <condition_variable>
#include <memory>
#include <optional>
#include <random>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <variant>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

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

class Explorer
{
 public:
	Explorer(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

	~Explorer();

 private:
	void terminate();

	bool runCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

	void odomCallback(nav_msgs::Odometry::ConstPtr const& msg);

	void obstacleAvoidanceFeedback(
	    collision_avoidance::PathControlFeedback::ConstPtr const& feedback);

	//
	// Main functions
	//

	void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg);

	void updateGraph(ufo::geometry::AABB region);

	void extendGraph(ufo::geometry::AABB region, size_t samples);

	std::vector<std::shared_ptr<Node>> getCandidateNodes() const;

	std::vector<std::vector<std::shared_ptr<Node>>> getCandidateClusters(
	    std::vector<std::shared_ptr<Node>> const& nodes, int eps, int min_pts,
	    std::vector<std::shared_ptr<Node>>* noise_cluster = nullptr) const;

	std::optional<ufo::math::Vector3> getCurrentPosition(
	    std_msgs::Header const& map_header) const;

	std::shared_ptr<Node> getCurrentNode(std_msgs::Header const& map_header) const;

	std::shared_ptr<Node> getNodeClosestCentroid(
	    std::vector<std::shared_ptr<Node>> const& nodes) const;

	ufo::math::Vector3 getCentroid(std::vector<std::shared_ptr<Node>> const& nodes) const;

	DistanceMatrix getDistanceMatrix(std::vector<std::shared_ptr<Node>> const& nodes) const;

	Tour getTour(DistanceMatrix const& distance_matrix) const;

	std::vector<std::shared_ptr<Node>> getPath(
	    std::shared_ptr<Node> const& current_position,
	    std::vector<std::shared_ptr<Node>> const& nodes) const;

	std::vector<std::shared_ptr<Node>> getPath(
	    std::shared_ptr<Node> const& current_position,
	    std::vector<std::vector<std::shared_ptr<Node>>> const& clusters,
	    std::vector<int> const& tour) const;

	std::vector<std::shared_ptr<Node>> shortenPath(
	    std::vector<std::shared_ptr<Node>> const& path,
	    ufo::math::Vector3 const& current_position) const;

	std::vector<std::shared_ptr<Node>> shortenPath(
	    std::vector<std::shared_ptr<Node>> const& path) const;

	void publishPath(std::vector<std::shared_ptr<Node>> const& path,
	                 std_msgs::Header const& map_header);

	void visualize(std_msgs::Header const& map_header,
	               std::vector<std::vector<std::shared_ptr<Node>>> const& clusters,
	               std::vector<std::shared_ptr<Node>> const& noise_cluster,
	               std::vector<std::shared_ptr<Node>> const& center_nodes,
	               std::vector<int> const& tour) const;

	void visualize(std_msgs::Header const& map_header) const;

	void printInfo() const;

	void writeComputationTimeToFile();

	//
	// Information gain thread
	//

	void gainWorkerThread();

	void updateGain(ufo::geometry::AABB region);

	//
	// Graph functions
	//

	void addEdge(std::shared_ptr<Node> const& node_1, std::shared_ptr<Node> const& node_2);

	void removeEdge(std::shared_ptr<Node> const& node_1,
	                std::shared_ptr<Node> const& node_2);

	void addNode(std::shared_ptr<Node> const& node);

	void deleteNode(std::shared_ptr<Node> const& node);

	std::shared_ptr<Node> getNearestNode(ufo::math::Vector3 const& position) const;

	std::vector<std::shared_ptr<Node>> getKNearestNodes(ufo::math::Vector3 const& position,
	                                                    int k) const;

	std::vector<std::shared_ptr<Node>> getNodesInRadius(ufo::math::Vector3 const& position,
	                                                    double radius) const;

	std::vector<std::shared_ptr<Node>> getNodesInBBX(ufo::math::Vector3 const& min,
	                                                 ufo::math::Vector3 const& max) const;

	Value getNearestValue(ufo::math::Vector3 const& position) const;

	std::vector<Value> getKNearestValues(ufo::math::Vector3 const& position, int k) const;

	std::vector<Value> getValuesInRadius(ufo::math::Vector3 const& position,
	                                     double radius) const;

	std::vector<Value> getValuesInBBX(ufo::math::Vector3 const& min,
	                                  ufo::math::Vector3 const& max) const;

	//
	// Helper functions
	//

	double dynamicPenalty(ufo::math::Vector3 const& from, ufo::math::Vector3 const& to,
	                      ufo::math::Vector3 const& vel) const;

	ufo::math::Vector3 restrictDistance(ufo::math::Vector3 const& position,
	                                    ufo::math::Vector3 const& closest) const;

	ufo::math::Vector3 samplePosition(ufo::math::Vector3 const& min,
	                                  ufo::math::Vector3 const& max);

	//
	// Clustering
	//

	std::vector<std::shared_ptr<Node>> dbscanRangeQuery(std::shared_ptr<Node> const& node,
	                                                    double yaw, int eps) const;

	std::vector<std::vector<std::shared_ptr<Node>>> dbscan(
	    std::vector<std::shared_ptr<Node>> const& nodes, int eps, int min_pts,
	    std::vector<std::shared_ptr<Node>>* noise_cluster = nullptr) const;

 private:
	// Subscribers
	ros::Subscriber map_sub_;
	ros::Subscriber odom_sub_;

	// Publishers
	ros::Publisher graph_pub_;
	ros::Publisher path_pub_;
	ros::Publisher global_tour_pub_;

	// Services
	ros::ServiceServer run_service_;

	// Actions
	actionlib::SimpleActionClient<collision_avoidance::PathControlAction> ac_;

	// TF
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	// Dynamic reconfigure
	// dynamic_reconfigure::Server<ufoexplorer::ExplorerConfig>* cs_;
	// dynamic_reconfigure::Server<ufoexplorer::ExplorerConfig>::CallbackType f_;

	// Graph
	ValueRtree graph_;
	// Max number of neighbors
	size_t max_num_neighbors_;
	// Max neighbor distance
	double max_neighbor_dist_;
	// Max edge length
	double max_edge_length_;

	bool running_ = false;

	// Clustering
	int eps_;      // Distance
	int min_pts_;  // Minimum number of points

	// Odometry
	ufo::math::Vector3 velocity_;
	double yaw_rate_;

	// Map
	UFOMapMutex map_;
	// Mutex for map
	mutable std::shared_mutex map_mutex_;
	// Automatic pruning
	bool automatic_pruning_;
	// Occupancy threshold
	double occupied_thres_;
	// Free threshold
	double free_thres_;

	// Depth used for collisions
	ufo::map::DepthType collision_depth_ = 0;
	// Depth used for gain
	ufo::map::DepthType gain_depth_ = 0;

	// Boundary
	ufo::geometry::AABB boundary_;

	// Calculate gain outside boundary
	bool continue_outside_;

	// Robot model
	RobotModel robot_;

	// Sensor model
	SensorModel sensor_;

	// Heuristic
	std::string heuristic_ = "closest";

	// Min gain threshold
	double min_gain_thres_;

	// Max yaw diff for cluster
	double max_yaw_diff_;

	// Maximum for exact TSP
	int max_exact_tsp_;

	// // Planning
	// std::thread planning_worker_;

	// Gain
	std::thread gain_worker_;
	std::mutex gain_mutex_;
	std::condition_variable gain_cv_;
	std::unordered_set<std::shared_ptr<Node>> gain_queue_;
	bool gain_done_ = false;

	// Random number generator
	std::random_device rd_;
	std::mt19937 gen_;                                  // std::default_random_engine
	std::uniform_real_distribution<double> real_dist_;  // std::normal_distribution<double>

	Timing timing_;

	bool exploration_started_ = false;

	bool exit_at_complete_;

	bool verbose_;

	std::ofstream output_file_;
	std::vector<std::string> component_names_;

	double total_volume_;
	double explorable_volume_;
	double explored_volume_ = 0.0;

	double max_time_;
};
}  // namespace ufoexplorer

#endif  // UFOEXPLORER_EXPLORER_H