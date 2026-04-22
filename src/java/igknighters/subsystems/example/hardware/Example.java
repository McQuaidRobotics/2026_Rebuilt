package igknighters.subsystems.example.hardware;

/**
 * Abstract base for the Example hardware.
 * Defines the functional methods that any implementation (Real, Sim, Disabled) must provide.
 */
public abstract class Example {
    /**
     * Sets the power of the mechanism.
     * @param speed The speed from -1.0 to 1.0.
     */
    public abstract void setSpeed(double speed);

    /**
     * Returns the current position of the mechanism.
     * @return Position in meters.
     */
    public abstract double getPosition();

    /**
     * Stops the mechanism.
     */
    public abstract void stop();
}
