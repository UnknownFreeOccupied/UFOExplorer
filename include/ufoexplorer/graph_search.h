#ifndef UFOEXPLORER_GRAPH_SEARCH_H
#define UFOEXPLORER_GRAPH_SEARCH_H

// UFO
#include <ufoexplorer/node.h>

// STL
#include <utility>
#include <vector>

namespace ufoexplorer
{
using Path = std::pair<std::vector<std::shared_ptr<Node>>, double>;

// struct Path {
// 	std::vector<std::shared_ptr<Node>> path;
// 	double cost;
// };

Path astar(std::shared_ptr<Node> const& start, std::shared_ptr<Node> const& goal,
           ufo::math::Vector3 velocity, bool reverse = true);

Path astar(std::shared_ptr<Node> const& start,
           std::vector<std::shared_ptr<Node>> const& goals, ufo::math::Vector3 velocity,
           bool reverse = true);

std::vector<Path> astar(std::vector<std::shared_ptr<Node>> const& starts,
                        std::shared_ptr<Node> const& goal, bool reverse = true);

// Bidirectional A* (A-star)
Path biAstar(std::shared_ptr<Node> const& start, std::shared_ptr<Node> const& goal,
             bool reverse = true);

}  // namespace ufoexplorer

#endif  // UFOEXPLORER_GRAPH_SEARCH_H