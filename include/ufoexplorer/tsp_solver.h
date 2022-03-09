#ifndef UFOEXPLORER_TSP_SOLVER_H
#define UFOEXPLORER_TSP_SOLVER_H

// UFO
#include <ufoexplorer/node.h>

// STL
#include <utility>
#include <vector>

namespace ufoexplorer
{
using DistanceMatrix = std::vector<std::vector<double>>;
using Tour = std::pair<std::vector<int>, double>;

// Naive
Tour TSPBruteForce(DistanceMatrix const& distance_matrix, int start = -1, int stop = -1,
                   bool open = true);

// Dynamic programming
// Tour TSPDP(DistanceMatrix const& distance_matrix, int start = -1, int stop = -1, bool
// open = true);

// Nearest-neighbor
Tour TSPNN(DistanceMatrix const& distance_matrix, int start = -1, int stop = -1,
           bool open = true);

// Christofides
Tour TSPChristofides(DistanceMatrix const& distance_matrix, int start = -1, int stop = -1,
                     bool open = true);

}  // namespace ufoexplorer

#endif  // UFOEXPLORER_TSP_SOLVER_H