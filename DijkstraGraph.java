import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.PriorityQueue;

/**
 * This class extends the BaseGraph data structure with additional methods for computing the total
 * cost and list of node data along the shortest path connecting a provided starting to ending
 * nodes. This class makes use of Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number> extends BaseGraph<NodeType, EdgeType>
    implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph. The final node in this path is stored in its node
     * field. The total cost of this path is stored in its cost field. And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in its node field).
     *
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;

        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }

        public int compareTo(SearchNode other) {
            if (cost > other.cost)
                return +1;
            if (cost < other.cost)
                return -1;
            return 0;
        }
    }

    /**
     * Constructor that sets the map that the graph uses.
     * @param map the map that the graph uses to map a data object to the node
     *        object it is stored in
     */
    public DijkstraGraph(MapADT<NodeType, Node> map) {
        super(map);
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations. The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *                                or when either start or end data do not
     *                                correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        // store nodes based on cost
        PriorityQueue<SearchNode> priorityQueue = new PriorityQueue<>();
        // store cost of nodes
        Map<NodeType, Double> cost = new HashMap<>();

        Node startNode = nodes.get(start);
        SearchNode startSearchNode = new SearchNode(startNode, 0, null);
        priorityQueue.add(startSearchNode);
        // if no weighted edge cost is zero
        cost.put(start, 0.0);
// loop goes until the priority queue is empty
        while (!priorityQueue.isEmpty()) {
            // get node with the lowest cost
            SearchNode current = priorityQueue.poll();
// check if the node is the end node
            if (current.node.data.equals(end)) {
                return current;
            }
// go to the next nodes
            for (Edge edge : current.node.edgesLeaving) {
                Node nextNode = edge.successor;
                double newCost = current.cost + edge.data.doubleValue();
// see if the cost is lower then last node 
                if (!cost.containsKey(nextNode.data)
                    || newCost < cost.get(nextNode.data)) {
                    SearchNode nextSearchNode = new SearchNode(nextNode, newCost, current);
                    priorityQueue.add(nextSearchNode);
            // update hashmap for cost
                    cost.put(nextNode.data, newCost);
                }
            }
        }

        // If nodes aren't connected with an edge or node doesn't exist through an error
        throw new NoSuchElementException();
    }


    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value. This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path. This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        // call the computeShortestPath method 
        SearchNode endNode = computeShortestPath(start, end);

        LinkedList<NodeType> path = new LinkedList<>();
   // start with the end node and go backwards
        SearchNode current = endNode;
        while (current != null) {
            try {
                // use reflection to access data of the node
                java.lang.reflect.Field dataField =
                    current.node.getClass().getDeclaredField("data");
                dataField.setAccessible(true);
                // add the data item to the front of the list
                path.addFirst((NodeType) dataField.get(current.node));
    // error handling 
            } catch (NoSuchFieldException | IllegalAccessException e) {
                e.printStackTrace();
            }
            // next go to the predecessor node and repeat
            current = current.predecessor;
        }
// return list of data item from node along this shortest path
        return path;
    }

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data. This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        // call the computeShortestPath method
        SearchNode endNode = computeShortestPath(start, end);
        // the cost of the shortest path between these nodes
        return endNode.cost;
    }
    }