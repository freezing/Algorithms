#include "graph.h"

#include <queue>

namespace algorithm {
namespace graph {

void Bfs(const Graph& graph, NodeIndex initial_node_index, std::function<bool(NodeIndex)> visitor) {
    // Ensure that the initial_node_index exists in the graph.
    assert(graph.nodes().find(initial_node_index) != graph.nodes().end());

    // Vector of visited nodes to ensure we don't process the same node more than once, i.e. ensure that it's not
    // inserted more than once in the queue.
    // Node u is visited if visited[u] == true.
    // Node u is not visited if visited[u] == false.
    std::vector<bool> visited(graph.capacity(), false);
    std::queue<NodeIndex> q;

    // Start with the initial node.
    q.push(initial_node_index);
    visited[initial_node_index] = true;

    while (!q.empty()) {
        // Take the first node index from the queue.
        NodeIndex u = q.front();
        q.pop();

        // Visit the node.
        bool keep_going = visitor(u);
        if (!keep_going) {
            return;
        }

        // Visit its neighbours.
        const std::vector<Edge>& edges = graph.edges(u);
        for (const auto& edge : edges) {
            NodeIndex v = edge.destination();
            if (!visited[v]) {
                // Mark node as visited and insert it in the queue.
                visited[v] = true;
                q.push(v);
            }
        }
    }
}

}
}