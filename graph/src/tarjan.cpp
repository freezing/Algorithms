#include <graph.h>
#include <stack>

namespace algorithm {
namespace graph {

namespace {
    const int32_t kUndefined = -1;

    struct TraversalData {
        int32_t index;
        int32_t lowest_link;

        TraversalData() : index(kUndefined), lowest_link(kUndefined) { }
    };
}

void strongConnect(
                   const Graph& graph,
                   NodeIndex node,
                   std::stack<NodeIndex>& stack,
                   std::vector<bool>& on_stack,
                   std::vector<TraversalData>& traversal_indexes,
                   int32_t& traversal_index,
                   StronglyConnectedComponent& scc) {
    // Set the traversal index of the node to the smallest available traversal_index.
    auto& rootTraversalData = traversal_indexes[node];
    rootTraversalData.index = traversal_index;
    rootTraversalData.lowest_link = traversal_index;
    traversal_index++;

    // Push the node on the stack.
    on_stack[node] = true;
    stack.push(node);

    // Iterate through successors of node.
    const auto& edges = graph.edges(node);
    for (const auto& edge : edges) {
        NodeIndex v = edge.destination();
        auto& successorTraversalData = traversal_indexes[v];
        if (successorTraversalData.index == kUndefined) {
            // Successor v hasn't been visited yet.
            strongConnect(graph, v, stack, on_stack, traversal_indexes, traversal_index, scc);
            rootTraversalData.lowest_link = std::min(rootTraversalData.lowest_link, successorTraversalData.lowest_link);
        } else if (on_stack[v]) {
            // Successor v is in stack and hence in the current SCC.
            // Note that at this point successorTraversalData.index and lowest_link are the same.
            rootTraversalData.lowest_link = std::min(rootTraversalData.lowest_link, successorTraversalData.index);
        }
        // else if v is not on the stack, then (node, v) is a cross-edge in the DFS tree and must be ignored.
    }

    // If node is the root node, pop the stack and generate SCC.
    if (rootTraversalData.lowest_link == rootTraversalData.index) {
        NodeIndex v;
        do {
            v = stack.top();
            stack.pop();
            on_stack[v] = false;
            scc.push_back(v);
        } while (v != node);
    }
}

std::vector<StronglyConnectedComponent> Tarjan(const Graph& graph) {
    int32_t traversal_index = 0;
    std::vector<StronglyConnectedComponent> sccs;
    std::vector<TraversalData> traversal_indexes(graph.capacity());
    std::stack<NodeIndex> stack;
    std::vector<bool> on_stack(graph.capacity(), false);
    auto& nodes = graph.nodes();

    for (NodeIndex node : nodes) {
        if (traversal_indexes[node].index == kUndefined) {
            StronglyConnectedComponent scc;
            strongConnect(graph, node, stack, on_stack, traversal_indexes, traversal_index, scc);
            if (!scc.empty()) {
                sccs.push_back(scc);
            }
        }
    }
    return sccs;
};

}
}