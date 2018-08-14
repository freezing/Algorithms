#include <graph.h>

namespace algorithm {
namespace graph {

std::vector<std::vector<Weight>> FloydWarshall(const Graph& graph) {
    // Initialize the distances matrix with the weights of the edges (use min weight if multiple edges exist),
    // or kInfinity if no edges between the nodes exist.
    std::vector<std::vector<Weight>> distances(graph.capacity(), std::vector<Weight>(graph.capacity(), kInfinity));
    const auto& nodes = graph.nodes();
    for (NodeIndex node : nodes) {
        // Initialize all distances from node to itself to 0.
        distances[node][node] = 0;

        const auto& edges = graph.edges(node);
        for (const auto& edge : edges) {
            distances[node][edge.destination()] = std::min(distances[node][edge.destination()], edge.weight());
        }
    }

    // Add intermediary vertices one by one in the set and compute the shortest distances between
    // all pairs of vertices such that the shortest distance considers vertices from the set {0, 1, 2, ..., V}
    // in each intermediary step.
    for (NodeIndex k = 0; k < graph.capacity(); k++) {
        for (NodeIndex i = 0; i < graph.capacity(); i++) {
            for (NodeIndex j = 0; j < graph.capacity(); j++) {
                // Calculate the shortest distance between nodes i and j using the new intermediary node k.
                if (distances[i][k] != kInfinity && distances[k][j] != kInfinity) {
                    distances[i][j] = std::min(distances[i][j], distances[i][k] + distances[k][j]);
                }
            }
        }
    }
    return distances;
};

}
}