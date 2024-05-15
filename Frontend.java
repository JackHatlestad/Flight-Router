import java.io.FileNotFoundException;
import java.util.NoSuchElementException;
import java.util.Scanner;

/**
 * This class exposes the functionality of the FrontendInterface to the Backend
 *
 * @author Chance Howarth
 *
 */
public class Frontend implements FrontendInterface {

    BackendInterface backend;
    Scanner inputScanner;

    /**
     * Constructor for Frontend class.
     *
     * @param backend The backend instance to be used with this frontend.
     */
    public Frontend(BackendInterface backend) {
        this.backend = backend;
    }

    /**
     * Main loop for command line interface.
     */
    @Override
    public void commandLoop() {

        // new scanner object
        inputScanner = new Scanner(System.in);
        // initialize values to
        boolean dataLoaded = false;
        boolean continueRunning = true;
        // calls method to load data and checks if true before moving on
        do {
            System.out.println(
                    "Hello! Please select an action below by entering the corresponding number.\n1: Load a data file: ");

            if (dataLoaded) {
                System.out.println("2: Show stats on airports, flights, and miles ");
                System.out
                        .println("3: Show the shortest route between start and destination airports ");
            }
            System.out.println("4: Exit");
            // checks the next input after user loads file sucessfully
            String input = inputScanner.nextLine();
            // takes the user input and loads the correct method
            switch (input) {
                case "1":
                    dataLoaded = loadDataFromFile();
                    break;
                case "2":
                    if (dataLoaded) {
                        showStatsCommand();
                    } else {
                        System.out.println("Please load the data file before using this option.");
                    }
                    break;
                case "3":
                    if (dataLoaded) {
                        findRouteCommand();
                    } else {
                        System.out.println(
                                "Please load the data file (Option 1) before using this option.");
                    }
                    break;
                case "4":
                    continueRunning = exitLoop();
                    break;
                default:
                    System.out.println("Please enter a valid integer between 1 and 4");
            }

        } while (continueRunning);

        inputScanner.close();
    }

    /**
     * Loads data from a file.
     *
     * @return true if data is loaded successfully, false otherwise.
     */
    @Override
    public boolean loadDataFromFile() {
        // checks for file name and then called the backend to load file for use
        System.out.print("Enter data file name: ");
        String filePath = inputScanner.nextLine();
        try {
            backend.readData(filePath);
        } catch (FileNotFoundException e) {
            System.out.println(filePath + " file not loaded");
            return false;
        }
        System.out.println(filePath + " file sucessfully loaded");
        return true;

    }

    /**
     * Displays statistics about airports, flights, and travel miles.
     */
    @Override
    public void showStatsCommand() {
        // prints stats out for user
        System.out
                .println("Here is info about the airports, flights and travel miles in our data set:");
        System.out.println(backend.getSetStats());
    }

    /**
     * Finds and displays the shortest route between two airports.
     */
    @Override
    public void findRouteCommand() {
        while (true) {
            // asks user for source airport and checks is true
            try {
                System.out
                        .print("Please enter a airport name abbreviation (ex. MKE, LAX)\nEnter Starting Airport: ");
                String startingAirport = inputScanner.nextLine();

                if (!backend.getGraph().containsNode(startingAirport)) {
                    System.out.println("Please enter a valid airport abbreviation");
                    continue;
                }
                // asks user for ending airport and checks if true
                System.out.print("Enter Ending Airport: ");
                String endingAirport = inputScanner.nextLine();

                if (!backend.getGraph().containsNode(endingAirport)) {
                    System.out.println("Please enter a valid airport abbreviation");
                    continue;
                }
                // calls backend getShortestRoute if both inputs are valid
                System.out.println(backend.getShortestRoute(startingAirport, endingAirport));
                break;
            } catch (NoSuchElementException e) {
                System.out.println("There is no route between airports");
                return;
            }
        }
    }

    /**
     * Exits the command loop.
     *
     * @return false to indicate the loop should terminate.
     */
    @Override
    public boolean exitLoop() {
        // exit loop when users inputs 4
        System.out.println("Thanks for using Flight Router, Goodbye!");
        return false;
    }

    /**
     * Sets a new backend instance.
     *
     * @param backend The new BackendInterface instance to be used.
     */
    public void setBackendInstance(BackendInterface backend) {
        this.backend = backend;
    }

    /**
     * Main method to start the application.
     *
     * @param args Command line arguments.
     */
    public static void main(String[] args) {
        Backend backend = new Backend(null);
        Frontend frontend = new Frontend(backend);
        frontend.setBackendInstance(backend);
        frontend.commandLoop();
    }
}