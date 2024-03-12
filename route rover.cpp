#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define MAX_NODES 100
#define INF INT_MAX

// Structure to represent edges
typedef struct Edge {
    int dest;             // Destination node of the edge
    int weight;           // Weight (distance) of the edge
    struct Edge* next;    // Pointer to the next edge in the list
} Edge;

// Structure to represent a graph
typedef struct Graph {
    Edge* edges[MAX_NODES]; // Array of edge pointers for each node
    int num_nodes;          // Number of nodes in the graph
} Graph;

// Function to initialize a graph with n nodes
Graph* init_graph(int n) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    graph->num_nodes = n;
    for (int i = 0; i < n; ++i) {
        graph->edges[i] = NULL; // Initialize all edge pointers to NULL
    }
    return graph;
}

// Function to add an edge to the graph
void add_edge(Graph* graph, int src, int dest, int weight) {
    Edge* edge = (Edge*)malloc(sizeof(Edge)); // Allocate memory for the new edge
    edge->dest = dest;       // Set destination node
    edge->weight = weight;   // Set edge weight
    edge->next = graph->edges[src]; // Insert the edge at the beginning of the list
    graph->edges[src] = edge;
}

// Function to perform Dijkstra's algorithm to find the shortest path
void dijkstra(Graph* graph, int start, int end, int* shortest_path) {
    int dist[MAX_NODES];    // Array to store distances from the start node
    int visited[MAX_NODES] = {0}; // Array to mark visited nodes

    // Initialize distances
    for (int i = 0; i < graph->num_nodes; ++i) {
        dist[i] = INF;  // Set all distances to infinity
    }
    dist[start] = 0;    // Set distance from start node to itself as 0

    // Main loop
    for (int count = 0; count < graph->num_nodes - 1; ++count) {
        int u = -1, min_dist = INF;
        // Find the vertex with minimum distance
        for (int v = 0; v < graph->num_nodes; ++v) {
            if (!visited[v] && dist[v] < min_dist) {
                min_dist = dist[v];
                u = v;
            }
        }
        // Mark the picked vertex as visited
        visited[u] = 1;
        // Update distances of the adjacent vertices
        for (Edge* edge = graph->edges[u]; edge != NULL; edge = edge->next) {
            int v = edge->dest;
            if (!visited[v] && dist[u] + edge->weight < dist[v]) {
                dist[v] = dist[u] + edge->weight;
                shortest_path[v] = u; // Record predecessor for path reconstruction
            }
        }
    }

    // Reconstruct shortest path
    int current = end;
    int path[MAX_NODES], path_length = 0;
    while (current != start) {
        path[path_length++] = current;
        current = shortest_path[current];
    }
    path[path_length++] = start;

    // Print shortest path
    printf("Shortest path from %d to %d: ", start, end);
    for (int i = path_length - 1; i >= 0; --i) {
        printf("%d", path[i]);
        if (i > 0) printf(" -> ");
    }
    printf("\nShortest distance: %d\n", dist[end]);
}

int main() {
    int num_nodes, num_edges;
    printf("Enter the number of nodes in the graph: ");
    scanf("%d", &num_nodes);
    Graph* graph = init_graph(num_nodes);

    printf("Enter the number of edges in the graph: ");
    scanf("%d", &num_edges);

    printf("Enter the edges in the format (source destination weight):\n");
    for (int Si = 0; i < num_edges; ++i) {
        int src, dest, weight;
        scanf("%d %d %d", &src, &dest, &weight);
        add_edge(graph, src, dest, weight);
    }

    int start, end;
    printf("Enter the start node: ");
    scanf("%d", &start);
    printf("Enter the end node: ");
    scanf("%d", &end);

    // Perform Dijkstra's algorithm
    int shortest_path[MAX_NODES];
    dijkstra(graph, start, end, shortest_path);

    // Free allocated memory
    for (int i = 0; i < graph->num_nodes; ++i) {
        Edge* edge = graph->edges[i];
        while (edge != NULL) {
            Edge* temp = edge;
            edge = edge->next;
            free(temp);
        }
    }
    free(graph);

    return 0;
}
