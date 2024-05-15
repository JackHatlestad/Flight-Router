import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This class exposes the functionality of the BackendInterface to the Frontend
 * 
 * @author John Hatlestad
 *
 */
public class Backend implements BackendInterface {

    private DijkstraGraph<String, Double> graph; // Field to store graph
    private double totalMiles; // Field to keep track of total miles in graph
    private FrontendInterface frontendInstance; // Field to set frontendInstance

    /**
     * Constructor for Backend Class
     * @param graph
     */
    public Backend(FrontendInterface frontendInstance) {
        this.totalMiles = 0.0;
        this.frontendInstance = frontendInstance;
        this.graph = new DijkstraGraph<>(new PlaceholderMap<>());
    }

    /**
     * Loads in a .Dot file and inserts it in a graph 
     * @param filepath - name of .Dot file provided by user
     * @throws FileNotFoundException if name of .Dot file doesn't exist  
     */
    public void readData(String filepath) throws FileNotFoundException {
        try (BufferedReader reader = new BufferedReader(new FileReader(filepath))) {
            String line;
            while ((line = reader.readLine()) != null) {
                // using regular expression to extract data
                Matcher dataExtract = Pattern
                    .compile("\"(\\w+)\"\\s*--\\s*\"(\\w+)\"\\s*\\[miles=(\\d+)\\]").matcher(line);

                if (dataExtract.find()) {
                    String startAirport = dataExtract.group(1);
                    String endAirport = dataExtract.group(2);
                    double weight = Double.parseDouble(dataExtract.group(3));

                    if (!graph.containsNode(startAirport)) {
                        graph.insertNode(startAirport);
                    }
                    if (!graph.containsNode(endAirport)) {
                        graph.insertNode(endAirport);
                    }
                    graph.insertEdge(startAirport, endAirport, weight);
                    totalMiles += weight;
                }
            }
        } catch (IOException e) {
            throw new FileNotFoundException("File not found: " + filepath);
        }
    }

    /**
     * Obtain the shortest route of given start airport and end airport as a string representation.
     * @param startAirport
     * @param endAirport
     * @return a string representation of the shortest route between the given airports
     */
    public String getShortestRoute(String startAirport, String endAirport)
        throws IllegalArgumentException {

        if (!graph.containsNode(startAirport) || !graph.containsNode(endAirport)) {
            throw new IllegalArgumentException();
        }
        // Use Dijkstra's algorithm to find the shortest path and total miles
        List<String> route = graph.shortestPathData(startAirport, endAirport);
        double totalMiles = graph.shortestPathCost(startAirport, endAirport);

        String routeString = "";
        // Retrieving the individual segment miles
        for (int i = 0; i < route.size() - 1; i++) {
            String airportOne = route.get(i);
            String airportTwo = route.get(i + 1);

            double segmentMiles = graph.getEdge(airportOne, airportTwo);
            routeString += airportOne + " (" + segmentMiles + " miles) -> ";
        }
        routeString += route.get(route.size() - 1) + " (Total Miles: " + totalMiles + ")";

        return routeString;
    }

    /**
     * get a string with statistics about the data set that includes 
     * the number of nodes (airports), 
     * the number of edges (flights), 
     * and the total miles (sum of weights) for all edges in the graph
     * @return a string represents the data statistics
     */
    public String getSetStats() {
        int numNodes = graph.getNodeCount();
        int numEdges = graph.getEdgeCount();

        return "The total number of airports is: " + numNodes + "\nThe total number of flights is: "
            + numEdges + " \nThe total miles is: " + this.totalMiles;
    }

    /**
    * Sets the FrontendInstance
    * @param frontend
    */
    public void setFrontendInstance(FrontendInterface frontend) {
        this.frontendInstance = frontend;
    }

    /**
     * getter method to retrieve the graph 
     * @return the loaded graph
     */
    public DijkstraGraph<String, Double> getGraph() {
        return graph;
    }

    /**
     * Main method to run the app and allow us to use the frontend 
     * @param args
     */
    public static void main(String[] args) {
        Frontend frontend = new Frontend(null);
        Backend backend = new Backend(frontend);
        frontend.setBackendInstance(backend);
        frontend.commandLoop();
    }
}