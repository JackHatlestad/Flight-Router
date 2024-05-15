import java.util.List;

public interface FlightPathInterface {
    
    /**
     * getter method to get the list of airports alone the route
     * @return list of string referred as airports
     */
    public List<String> getRoute();

    /**
     * getter method to get a list of miles to travel for each segments of the route
     * @return list of double referred as miles
     */
    public List<Double> getSegments();

    /**
     * getter method to get the total miles to travel
     * @return double that referred as miles
     */
    public double getTotalMiles();
}