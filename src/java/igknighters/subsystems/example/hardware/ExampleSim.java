package igknighters.subsystems.example.hardware;

/**
 * Simulation implementation for the Example subsystem.
 */
public class ExampleSim extends Example {
    private double currentPosition = 0.0;
    private double currentSpeed = 0.0;

    public ExampleSim() {
        // Initialize sim state here
    }

    @Override
    public void setSpeed(double speed) {
        this.currentSpeed = speed;
    }

    @Override
    public double getPosition() {
        // Simple simulation update
        currentPosition += currentSpeed * 0.02; // Assuming 20ms loops
        return currentPosition;
    }

    @Override
    public void stop() {
        setSpeed(0.0);
    }
}
