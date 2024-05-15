import java.util.List;

public class FlightPath implements FlightPathInterface {

    private final List<String> route;
    private final List<Double> segments;
    private final double totalMiles;

    public FlightPath(List<String> route, List<Double> segments, double totalMiles) {
        this.route = route;
        this.segments = segments;
        this.totalMiles = totalMiles;
    }

    @Override
    public List<String> getRoute() {
        return route;
    }

    @Override
    public List<Double> getSegments() {
        return segments;
    }

    @Override
    public double getTotalMiles() {
        return totalMiles;
    }
}