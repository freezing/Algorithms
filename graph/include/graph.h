#ifndef ALGORITHMS_GRAPH_H
#define ALGORITHMS_GRAPH_H

#include <memory>
#include <vector>
#include <set>
#include <map>
#include <functional>
#include <limits>
#include <cassert>

namespace algorithm {
namespace graph {

using NodeIndex = uint32_t;
using StronglyConnectedComponent = std::vector<NodeIndex>;
using Weight = int32_t;
using ErrorCode = int32_t;

namespace {
    const Weight kUnweighted = 1;
    const Weight kInfinity = std::numeric_limits<Weight>::max();
}

namespace {
    const ErrorCode kSuccess = 0;
    const ErrorCode kGraphNotDag = 1;
}

struct Edge {
private:
    Weight _weight;
    NodeIndex _source;
    NodeIndex _destination;

public:
    Edge() { }

    Edge(NodeIndex source, NodeIndex destination, Weight weight)
            : _source(source), _destination(destination), _weight(weight) { }

    // Returns weight of the edge.
    inline Weight weight() const {
        return _weight;
    }

    // Returns the source node, i.e. the node that the edge is going out from.
    inline NodeIndex source() const {
        return _source;
    }

    // Returns the destination node, i.e. the node that the edge is going to.
    inline NodeIndex destination() const {
        return _destination;
    }

};

// A graph data-structure with nodes indexed from [0, capacity).
class Graph {
private:
    // Total available capacity of the graph.
    uint32_t _capacity;

    // Set of nodes in the graph.
    std::set<NodeIndex> _nodes;

    // A pointer to the array of vectors that represent edges.
    // Vector at index u contains edges from node u.
    std::vector<Edge> *_edges;

public:
    // Creates a graph with the maximum specified capacity.
    Graph(uint32_t capacity) : _capacity(capacity) {
        _edges = new std::vector<Edge>[_capacity];
    }

    ~Graph() {
        delete[] _edges;
    }


    // Returns the total capacity of the graph.
    //
    // Time complexity: O(1).
    uint32_t capacity() const {
        return _capacity;
    }

    // Adds node to the graph. This method is noop if node already exists.
    //
    // Time complexity: O(1).
    void addNode(NodeIndex u) {
        assert(u < capacity);
        _nodes.insert(u);
    }

    // Adds a directed weighted edge from node u to node v with weight w.
    // It adds new edge even if the edge from u to v already exists.
    // Adds nodes if they do not already exist.
    //
    // Time complexity: O(1).
    void addEdge(NodeIndex u, NodeIndex v, Weight w) {
        _edges[u].emplace_back(u, v, w);
        addNode(u);
        addNode(v);
    }

    // Adds a directed unweighted edge from node u to node v.
    // It adds new edge even if the edge from u to v already exists.
    // Adds nodes if they do not already exist.
    //
    // Time complexity: O(1).
    void addEdge(NodeIndex u, NodeIndex v) {
        addEdge(u, v, kUnweighted);
    }

    // Adds an undirected, unweighted edge from node u to node v.
    // The result of this method is the same as calling addEdge method twice with (u, v) and (v, u) as parameters.
    //
    // Time complexity O(1).
    void addUndirectedEdge(NodeIndex u, NodeIndex v) {
        addEdge(u, v);
        addEdge(v, u);
    }

    // Adds an undirected weighted edge from node uto node v with weight w.
    // The result of this method is the same as calling addEdge method twice with (u, v, w) and (v, u, w) as parameters.
    //
    // Time complexity: O(1).
    void addUndirectedEdge(NodeIndex u, NodeIndex v, Weight w) {
        addEdge(u, v, w);
        addEdge(v, u, w);
    }

    // Returns a const reference to the list of edges from the node with index u.
    //
    // Time complexity: O(1).
    const std::vector<Edge>& edges(NodeIndex u) const {
        return _edges[u];
    }

    // Returns a const reference to the list of nodes in the graph.
    //
    // Time complexity: O(1).
    const std::set<NodeIndex>& nodes() const {
        return _nodes;
    }
};

// Traverses the graph using breadth-first-search algorithm starting at the specified node index
// and calls the supplied callback function each time
// it visits new node.
// The algorithm finishes when the whole graph is traversed or the callback function returns false.
// It is caller's responsibility to ensure that the initial node exists in the graph.
//
// V - graph capacity.
// E - number of edges.
// O(C) - time complexity of the callback method.
// Time complexity: O(V * C + E).
// Memory complexity: O(V).
void Bfs(const Graph&, NodeIndex, std::function<bool(NodeIndex)>);

// Tarjan's algorithm for finding strongly connected components:
// https://en.wikipedia.org/wiki/Tarjan%27s_strongly_connected_components_algorithm
//
// Returns a vector of StronglyConnectedComponents, where each SCC is a vector of disjunctive node indexes.
//
// V - graph capacity.
// E - number of edges.
// Time complexity: O(V + E).
// Memory complexity: O(V).
std::vector<StronglyConnectedComponent> Tarjan(const Graph&);

// Dijkstra's algorithm for finding the shortest paths from the given node to all other nodes in the graph.
// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
//
// Returns a vector d of distances that represent shortest paths from the source node,
// where d[u] is the distance to the node u.
// Note that the size of the vector is graph.capacity() and the indexes for nodes that do not exist in the graph
// have value of kInfinity.
// If a node exists in the graph but doesn't exist as the key in the map, then there is no path to that node.
//
// V - graph capacity.
// E - number of edges.
// Time complexity: O(V + E + lg (V + E)).
// Memory complexity: O(V).
std::vector<Weight> Dijkstra(const Graph&, NodeIndex);

// Floyd Warshall algorithm for finding the shortest paths for all pairs in a given weighted directed graph.
// https://en.wikipedia.org/wiki/Floyd%E2%80%93Warshall_algorithm
//
// Returns a matrix D with value d[u][v] being equal to the length of the shortest path from node u to node v.
//
// V - graph capacity.
// E - number of edges.
// Time complexity: O(V^3 + E).
// Memory complexity: O(V^2).
std::vector<std::vector<Weight>> FloydWarshall(const Graph&);

// DFS based topological sorting algorithm:
// https://en.wikipedia.org/wiki/Topological_sorting
//
// Returns a pair of result and error code, where result is vector of node indexes sorted in topological order.
// Error code is kSuccess and result is populated if the graph is DAG.
// Error code is kGraphNotDag if the graph is not DAG.
//
// V - graph capacity.
// E - number of edges.
// Time complexity: O(V + E).
// Memory complexity: O(V).
std::pair<std::vector<NodeIndex>, ErrorCode> TopologicalSort(const Graph&);

}
}

#endif //ALGORITHMS_GRAPH_H
