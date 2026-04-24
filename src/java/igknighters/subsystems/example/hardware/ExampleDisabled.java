package igknighters.subsystems.example.hardware;

/**
 * Disabled implementation for the Example subsystem. Useful for debugging logic when the hardware
 * isn't needed.
 */
public class ExampleDisabled extends Example {

    @Override
    public void setSpeed(double speed) {
        // No-op
    }

    @Override
    public double getPosition() {
        return 0.0;
    }

    @Override
    public void stop() {
        // No-op
    }
}
