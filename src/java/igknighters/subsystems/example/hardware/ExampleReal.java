package igknighters.subsystems.example.hardware;

/** Real hardware implementation for the Example subsystem. */
public class ExampleReal extends Example {

    public ExampleReal(int motorId) {
        // Initialize real hardware here
    }

    @Override
    public void setSpeed(double speed) {
        // Apply speed to the real motor
    }

    @Override
    public double getPosition() {
        // Return real sensor position
        return 0.0;
    }

    @Override
    public void stop() {
        setSpeed(0.0);
    }
}
