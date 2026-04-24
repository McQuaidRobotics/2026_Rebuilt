package igknighters.subsystems.example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.example.hardware.Example;
import igknighters.subsystems.example.hardware.ExampleDisabled;
import igknighters.subsystems.example.hardware.ExampleReal;
import igknighters.subsystems.example.hardware.ExampleSim;
import org.littletonrobotics.junction.Logger;

/**
 * The Example Subsystem. Orchestrates the hardware implementations and provides high-level logic.
 *
 * <p>This follows the project's pattern of abstracting hardware (Real, Sim, Disabled) into a
 * separate component class while the subsystem handles coordination and logging.
 */
public class ExampleSubsystem extends SubsystemBase {
    private final Example hardware;

    /**
     * Constructs the subsystem, initializing the appropriate hardware implementation.
     *
     * @param isDisabled If true, uses the Disabled implementation regardless of robot state.
     */
    public ExampleSubsystem(boolean isDisabled) {
        if (isDisabled) {
            hardware = new ExampleDisabled();
        } else if (Robot.isReal()) {
            hardware = new ExampleReal(1); // Real motor ID
        } else {
            hardware = new ExampleSim();
        }
    }

    @Override
    public void periodic() {
        // Log mechanism data for debugging and AdvantageScope.
        Logger.recordOutput("Example/Position", hardware.getPosition());
    }

    /**
     * High-level method to move the mechanism.
     *
     * @param speed The speed from -1.0 to 1.0.
     */
    public void runAtSpeed(double speed) {
        hardware.setSpeed(speed);
    }

    /** Stops the mechanism. */
    public void stop() {
        hardware.stop();
    }
}
