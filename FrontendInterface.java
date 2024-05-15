import java.io.FileNotFoundException;

public interface FrontendInterface {



    /*
     * First method that will start the main commandLoop.
     */
    public void commandLoop();

    /*
     * Method to load the data from the file.
     */
    public boolean loadDataFromFile() throws FileNotFoundException;

    /*
     * Method to show the statistics about the dataset that includes number of airports (nodes), the
     * number of edges (flight s), and the total number of miles (sum of all edge weights) in the
     * graph,
     */
    public void showStatsCommand();

    /*
     * Method that asks the user for a start and destination airport, then lists the shortest route
     * between those airports, including all airports on the way, the distance for each segment, and
     * the total number of miles from start to destination airport.
     */

    public void findRouteCommand();

    /*
     * Method to exit the loop
     */

    public boolean exitLoop();

}