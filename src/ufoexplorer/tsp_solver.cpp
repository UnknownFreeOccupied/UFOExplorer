// UFO
#include <ufoexplorer/tsp_solver.h>

// STL
#include <algorithm>
#include <iterator>
#include <random>

namespace ufoexplorer
{
double getTourLength(std::vector<int> const& tour, DistanceMatrix const& distance_matrix)
{
	double tour_length = 0.0;
	for (size_t index = 1; index < tour.size(); ++index) {
		int node = tour[index];
		int prev_node = tour[index - 1];
		tour_length += distance_matrix[prev_node][node];
	}
	// Only if should return to where it started
	// tour_length += distance_matrix[0][tour.size() - 1];

	return tour_length;
}

Tour TSPBruteForce(DistanceMatrix const& distance_matrix, int start, int stop, bool open)
{
	std::vector<int> ret;
	ret.reserve(distance_matrix.size());
	for (int i = 0; i < distance_matrix.size(); ++i) {
		if (start != i && stop != i) {
			ret.push_back(i);
		}
	}

	std::vector<int> best_tour;
	double best_tour_length = std::numeric_limits<double>::max();

	do {
		double tentative_tour_length = getTourLength(ret, distance_matrix);

		if (0 <= start) {
			tentative_tour_length += distance_matrix[start][ret.front()];
		}
		if (0 <= stop) {
			tentative_tour_length += distance_matrix[ret.back()][stop];
		}

		if (!open) {
			// Add extra cost for moving from end to start, to close the loop
			if (0 <= start && 0 <= stop) {
				tentative_tour_length += distance_matrix[stop][start];
			} else if (0 <= start) {
				tentative_tour_length += distance_matrix[ret.back()][start];
			} else if (0 <= stop) {
				tentative_tour_length += distance_matrix[stop][ret.front()];
			}
		}

		if (best_tour_length > tentative_tour_length) {
			best_tour_length = tentative_tour_length;
			best_tour = ret;
		}
	} while (std::next_permutation(ret.begin(), ret.end()));

	if (0 <= start) {
		best_tour.insert(best_tour.begin(), start);
	}
	if (0 <= stop) {
		best_tour.push_back(stop);
	}

	return std::make_pair(best_tour, best_tour_length);
}

// Tour TSPDP(DistanceMatrix const& distance_matrix, int start, int stop, bool open)
// {
// 	// TODO: Implement
// 	std::vector<int> ret;
// 	ret.reserve(distance_matrix.size());
// 	for (int i = 0; i < distance_matrix.size(); ++i) {
// 		if (start != i && stop != i) {
// 			ret.push_back(i);
// 		}
// 	}

// 	std::vector<bool> visited(ret.size(), false);
// 	double cost = 0;

// 	std::vector<int> best_tour;
// 	double best_tour_length = std::numeric_limits<double>::max();

// 	do {
// 		double tentative_tour_length = getTourLength(ret, distance_matrix);

// 		if (0 <= start) {
// 			tentative_tour_length += distance_matrix[start][ret.front()];
// 		}
// 		if (0 <= stop) {
// 			tentative_tour_length += distance_matrix[ret.back()][stop];
// 		}

// 		if (best_tour_length > tentative_tour_length) {
// 			best_tour_length = tentative_tour_length;
// 			best_tour = ret;
// 		}
// 	} while (std::next_permutation(ret.begin(), ret.end()));

// 	if (0 <= start) {
// 		best_tour.insert(best_tour.begin(), start);
// 	}
// 	if (0 <= stop) {
// 		best_tour.push_back(stop);
// 	}

// 	return std::make_pair(best_tour, best_tour_length);
// }

// Tour TSPDPHelper(std::vector<int> const& nodes, int start, std::vector<bool>& visited,
//                  std::vector<std::vector<double>> const& distance_matrix)
// {
// 	// TODO: Implement
// 	visited[start] = true;

// 	if (2 == nodes.size() &&) {
// 		// TODO: Implement
// 	} else {
// 		for (int j : nodes) {
// 			if (j == start) {
// 				continue;
// 			}
// 			for (int i : nodes) {
// 				if (j != i && !visited[i]) {
// 					std::vector<int> new_nodes = nodes;
// 					new_nodes.erase(std::remove(new_nodes.begin(), new_nodes.end(), i),
// 					                new_nodes.end());
// 					cost = TSPDPHelper(new_nodes, j) + distance_matrix[j][i];
// 					visited[j] = true;
// 				}
// 			}
// 		}
// 	}
// }

template <typename T>
void eraseElement(std::vector<T>& vec, T const& elem)
{
	vec.erase(std::remove(vec.begin(), vec.end(), elem), vec.end());
}

Tour TSPNN(DistanceMatrix const& distance_matrix, int start, int stop, bool open)
{
	std::vector<int> states(distance_matrix.size());
	std::generate(states.begin(), states.end(), [n = 0]() mutable { return n++; });

	std::vector<int> tour;
	tour.reserve(distance_matrix.size());

	if (0 <= stop) {
		// Should add stop at the end
		eraseElement(states, stop);
	}

	if (0 <= start) {
		tour.push_back(start);
		eraseElement(states, start);
	} else {
		// Take a random
		std::vector<int> samples;
		std::sample(states.begin(), states.end(), std::back_inserter(samples), 1,
		            std::mt19937{std::random_device{}()});
		tour.push_back(samples.front());
		eraseElement(states, samples.front());
	}

	while (!states.empty()) {
		// Find closest to previous
		int closest;
		double closest_dist = std::numeric_limits<double>::max();
		for (int state : states) {
			if (distance_matrix[tour.back()][state] < closest_dist) {
				closest_dist = distance_matrix[tour.back()][state];
				closest = state;
			}
		}
		tour.push_back(closest);
		eraseElement(states, closest);
	}

	if (0 <= stop) {
		tour.push_back(stop);
	}

	double tour_length = 0.0;
	for (int i = 1; i < tour.size(); ++i) {
		tour_length = distance_matrix[tour[i - 1]][tour[i]];
	}

	if (!open) {
		// Add extra cost for moving from end to start, to close the loop
		tour_length += distance_matrix[tour.back()][tour.front()];
	}

	return std::make_pair(tour, tour_length);
}

int findSet(std::vector<std::pair<int, int>>& parent, int i)
{
	if (parent[i].first != i) {
		parent[i].first = findSet(parent, parent[i].first);
	}
	return parent[i].first;
}

void unionSet(std::vector<std::pair<int, int>>& parent, int x, int y)
{
	int x_root = findSet(parent, x);
	int y_root = findSet(parent, y);

	if (parent[x_root].second < parent[y_root].second) {
		parent[x_root].first = y_root;
	} else if (parent[x_root].second > parent[y_root].second) {
		parent[y_root].first = x_root;
	} else {
		parent[y_root].first = x_root;
		parent[x_root].second++;
	}
}

// Kuskal's minimum spanning tree
std::vector<std::pair<int, int>> MST(DistanceMatrix const& distance_matrix)
{
	std::vector<std::pair<double, std::pair<int, int>>> edges_with_cost;
	for (int i = 0; i < distance_matrix.size(); ++i) {
		for (int j = 0; j < distance_matrix[i].size(); ++j) {
			if (i == j) {
				continue;
			}
			edges_with_cost.push_back(
			    std::make_pair(distance_matrix[i][j], std::make_pair(i, j)));
		}
	}
	std::sort(edges_with_cost.begin(), edges_with_cost.end());

	std::vector<std::pair<int, int>> parent(distance_matrix.size());
	std::generate(parent.begin(), parent.end(),
	              [n = 0]() mutable { return std::make_pair(n++, 0); });

	std::vector<std::pair<int, int>> mst;

	for (auto const& [cost, edge] : edges_with_cost) {
		int u_rep = findSet(parent, edge.first);
		int v_rep = findSet(parent, edge.second);
		if (u_rep != v_rep) {
			mst.push_back(edge);
			unionSet(parent, u_rep, v_rep);
		}
	}

	return mst;
}

void TSPAddDummyVertex(DistanceMatrix& distance_matrix, int start, int stop)
{
	if (0 > start && 0 > stop) {
		// Nothing to do
		return;
	}

	if (0 <= start && 0 <= stop) {
		// Add dummy vertex
		distance_matrix.push_back(
		    std::vector<double>(distance_matrix.size(), std::numeric_limits<double>::max()));
		for (std::vector<double>& x : distance_matrix) {
			x.push_back(std::numeric_limits<double>::max());
		}
		distance_matrix.back().back() = 0.0;

		for (int i = 0; i < distance_matrix.size() - 1; ++i) {
			if (i == start || i == stop) {
				distance_matrix[i].back() = 0.0;
				distance_matrix.back()[i] = 0.0;
			}
		}
	} else if (0 <= start) {
		// Set cost of moving to start to zero
		for (int i = 0; i < distance_matrix.size(); ++i) {
			distance_matrix[i][start] = 0.0;
		}
	} else if (0 <= stop) {
		// Set cost of moving from stop to zero
		for (int i = 0; i < distance_matrix.size(); ++i) {
			distance_matrix[stop][i] = 0.0;
		}
	}
}

Tour TSPChristofides(DistanceMatrix const& distance_matrix, int start, int stop,
                     bool open)
{
	DistanceMatrix dm_copy = distance_matrix;

	TSPAddDummyVertex(dm_copy, start, stop);

	std::vector<std::pair<int, int>> mst = MST(dm_copy);

	std::vector<int> degree(dm_copy.size(), 0);

	for (auto const& [s, t] : mst) {
		degree[s]++;
		degree[t]++;
	}

	std::vector<int> odd_degree;
	for (int i = 0; i < degree.size(); ++i) {
		if (0 != degree[i] % 2) {
			odd_degree.push_back(i);
		}
	}

	

	std::vector<int> tour;
	tour.reserve(distance_matrix.size());

	if (0 > start && 0 > stop) {
	} else if (0 > start) {
	} else if (0 > stop) {
	}

	double tour_length = 0.0;
	for (int i = 1; i < tour.size(); ++i) {
		tour_length = distance_matrix[tour[i - 1]][tour[i]];
	}

	if (!open) {
		// Add extra cost for moving from end to start, to close the loop
		tour_length += distance_matrix[tour.back()][tour.front()];
	}

	return std::make_pair(tour, tour_length);
}
}  // namespace ufoexplorer