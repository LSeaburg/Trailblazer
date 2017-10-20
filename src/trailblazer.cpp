/* Logan Seaburg
 *
 * To write this program I used lecture notes from class and
 * programming references on Stanford's C++ website
 */

#include "hashset.h"
#include "trailblazer.h"
#include "pqueue.h"
#include "queue.h"

using namespace std;

bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*>& path);
void traceBackPath(Vertex* start, Vertex* curr, Vector<Vertex*>& path);

/* This is a recursive function that searches a graph in DFS. It has a helper
 * function that is called to keep track of the path to that spot. It looks at
 * all the neighbors of a square and makes a recursive call for every neighbor that
 * is not visited.
 */
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    Vector<Vertex*> path;
    graph.resetData();
    path.add(start);
    start->visited = true;
    start->setColor(GREEN);
    if (!depthFirstSearchHelper(graph, start, end, path)) {
        path.clear();
        start->setColor(GRAY);
    }
    return path;
}
bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*>& path) {
    if (start == end) {
        return true;
    }
    for (Vertex* neighbor : graph.getNeighbors(start)) {
        if (!neighbor->visited) {
            neighbor->visited = true;
            neighbor->setColor(GREEN);
            path.add(neighbor);
            if (depthFirstSearchHelper(graph, neighbor, end, path)) {
                return true;
            }
            path.remove(path.size() - 1);
            neighbor->setColor(GRAY);
        }
    }
    return false;
}

/* Adds all of the surrounding squares into a queue and searches all of those
 * squares in waves so that the first path to reach the end is the shortest
 * path that exists
 */
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    Vector<Vertex*> path;
    Queue<Vertex*> toCheck;
    Vertex* curr;
    start->setColor(GREEN);
    graph.resetData();
    toCheck.enqueue(start);
    while (!toCheck.isEmpty() && toCheck.peek() != end) {
        curr = toCheck.dequeue();
        for (Vertex* neighbor : graph.getNeighbors(curr)) {
            if (!neighbor->visited) {
                neighbor->setColor(YELLOW);
                neighbor->visited = true;
                toCheck.enqueue(neighbor);
                neighbor->previous = curr;
            }
        }
        curr->setColor(GREEN);
    }
    if (!toCheck.isEmpty()) {
        traceBackPath(start, end, path);
        end->setColor(GREEN);
    }
    return path;
}

/* When all the previous of points from a start to and end has been set
 * this function will return all the vectors from the start to the end
 * as a vector passed by reference.
 */
void traceBackPath(Vertex* start, Vertex* curr, Vector<Vertex*>& path) {
    Stack<Vertex*> reverse;
    reverse.push(curr);
    while (curr != start) {
        curr = curr->previous;
        reverse.push(curr);
    }
    while (!reverse.isEmpty()) {
        path.add(reverse.pop());
    }
}

/* This function is very similar to the BFS, but it uses a priority ranked by
 * the cost of the square instead of a regular queue.
 */
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    Vector<Vertex*> path;
    PriorityQueue<Vertex*> toCheck;
    Vertex* curr;
    graph.resetData();
    start->setColor(GREEN);
    start->visited = true;
    toCheck.enqueue(start, 0);
    while (!toCheck.isEmpty() && toCheck.peek() != end) {
        curr = toCheck.dequeue();
        for (Vertex* neighbor : graph.getNeighbors(curr)) {
            if (!neighbor->visited ||
                    neighbor->cost > curr->cost + graph.getEdge(curr, neighbor)->cost) {
                neighbor->setColor(YELLOW);
                neighbor->visited = true;
                double cost = curr->cost + graph.getEdge(curr, neighbor)->cost;
                toCheck.enqueue(neighbor, cost);
                neighbor->cost = cost;
                neighbor->previous = curr;
            }
        }
        curr->setColor(GREEN);
    }
    if (!toCheck.isEmpty()) {
        traceBackPath(start, end, path);
        end->setColor(GREEN);
    }
    return path;
}

/* Very similar to Dijkstra's Algorithm but the priority queue is weighted on
 * the value the heurstic function returns instead of the current cost of the path.
 */
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    Vector<Vertex*> path;
    PriorityQueue<Vertex*> toCheck;
    Vertex* curr;
    graph.resetData();
    start->setColor(GREEN);
    start->visited = true;
    start->cost = 0;
    toCheck.enqueue(start, 0);
    while (!toCheck.isEmpty() && toCheck.peek() != end) {
        curr = toCheck.dequeue();
        for (Vertex* neighbor : graph.getNeighbors(curr)) {
            if (!neighbor->visited || neighbor->cost > curr->cost + graph.getEdge(curr, neighbor)->cost) {
                neighbor->setColor(YELLOW);
                neighbor->visited = true;
                neighbor->cost = curr->cost + graph.getEdge(curr, neighbor)->cost;
                toCheck.enqueue(neighbor, neighbor->cost + heuristicFunction(neighbor, end));
                neighbor->previous = curr;
            }
        }
        curr->setColor(GREEN);
    }
    if (!toCheck.isEmpty()) {
        traceBackPath(start, end, path);
        end->setColor(GREEN);
    }
    return path;
}

/* Keeps track of groups of connected vertices and only adds the lowest cost edges
 * if there is not already a connection between them. Repeats this until all of the
 * vertices are connected. Uses a map to keep track of the which set each vertex is in
 * to improve runtime significantly.
 */
Set<Edge*> kruskal(BasicGraph& graph) {
    Set<Edge*> mst;
    HashMap<int, HashSet<Vertex*> > vertices;
    HashMap<Vertex*, int> vertexMap;
    PriorityQueue<Edge*> edges;
    int i = 0;
    for(Vertex* vertex : graph.getVertexSet()) {
        HashSet<Vertex*> temp;
        temp.add(vertex);
        vertices.add(i, temp);
        vertexMap.add(vertex, i);
        i++;
    }
    for(Edge* edge : graph.getEdgeSet()) {
        edges.add(edge, edge->cost);
    }
    while(vertices.size() > 1) {
        Edge* e = edges.dequeue();
        Vertex* v1 = e->start;
        Vertex* v2 = e->end;
        int start = vertexMap[v1];
        int end = vertexMap[v2];
        if (start != end) {
            mst.add(e);
            for(Vertex* v : vertices[end]) {
                vertexMap[v] = start;
            }
            vertices[start] += vertices[end];
            vertices.remove(end);
        }
    }
    return mst;
}
