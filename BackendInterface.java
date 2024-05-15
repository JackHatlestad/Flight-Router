import java.io.FileNotFoundException;

public interface BackendInterface {

    /*
        * GraphADT<String, Double> graph;
        * 
        * public Backend(GraphADT<String, Double> graph){
        *      this.graph = graph;
        * }
        */

    /**
     * read the data with the given filepath 
     * @param filepath
     */
    public void readData(String filepath) throws FileNotFoundException;
    
    /**
     * obtain the shortest route of given start airport and end airport
     * @param startAirport
     * @param endAirport
     * @return a Route object that represents the shortest route between the given airports
     */
    public String getShortestRoute(String startAirport, String endAirport) throws IllegalArgumentException;
    
    /**
     * get a string with statistics about the data set that includes 
     * the number of nodes (airports), 
     * the number of edges (flights), 
     * and the total miles (sum of weights) for all edges in the graph
     * @return a string represents the data statistics
     */
    public String getSetStats();
    
   /**
    * getter method to retrieve the graph 
    * @return the loaded graph
    */
    public DijkstraGraph<String, Double> getGraph();

}