#include <graph.h>
#include <queue>
#include <vector>
#include <map>

namespace algorithm {
namespace graph {

class ReachedNode {
public:
    NodeIndex index;
    Weight distance;

    ReachedNode(NodeIndex node, Weight distance) : index(node), distance(distance) { }
};

class SmallestTopComparator {
public:
    bool operator() (const ReachedNode& a, const ReachedNode& b) const {
        // This means that the node with a lowest value will always be on top of the priority queue.
        return a.distance > b.distance;
    }
};

std::vector<Weight> Dijkstra(const Graph& graph, NodeIndex initialNode) {
    const auto& nodes = graph.nodes();
    assert(nodes.find(initialNode) != nodes.end());
    std::vector<Weight> distances(graph.capacity(), kInfinity);
    std::priority_queue<ReachedNode, std::vector<ReachedNode>, SmallestTopComparator> pq;

    // Reach the initial node.
    distances[initialNode] = 0;
    pq.emplace(initialNode, 0);

    while (!pq.empty()) {
        // Remove all the reached nodes with a distance greater than the shortest found distance.
        while (!pq.empty() && distances[pq.top().index] < pq.top().distance) {
            pq.pop();
        }

        // It's possible that the priority_queue is empty at this stage, so check and exit if so.
        if (pq.empty()) {
            continue;
        }

        // At this point we know that the smallest distance to the node at the top of the priority queue is
        // the smallest one, hence we are updating it.
        ReachedNode node = pq.top();
        pq.pop();

        // Visit all the neighbours with a shorter path than the one we've found so far.
        const auto& edges = graph.edges(node.index);
        for (auto& edge : edges) {
            Weight current_distance = distances[edge.destination()];
            Weight new_distance = distances[node.index] + edge.weight();
            if (current_distance > new_distance) {
                // We have found a shorter path to the destination node.
                // Insert it in the priority queue and update the best distance found so far.
                distances[edge.destination()] = new_distance;
                pq.emplace(edge.destination(), new_distance);
            }
        }
    }
    return distances;
}
}
}
