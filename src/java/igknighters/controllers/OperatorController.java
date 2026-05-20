package igknighters.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.IndexerCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.indexer.IndexerState;
import java.util.function.DoubleSupplier;

public class OperatorController {

    private final CommandJoystick flightStick;
    private final CommandJoystick flightThrottle;
    /** Right Trigger; Axis: 3 */
    protected final Trigger RT;

    /** DPad Up; Degrees: 0 */
    protected final Trigger DPU;

    /** DPad Right; Degrees: 90 */
    protected final Trigger DPR;

    /** DPad Down; Degrees: 180 */
    protected final Trigger DPD;

    /** DPad Left; Degrees: 270 */
    protected final Trigger DPL;

    public OperatorController(int port) {
        DriverStation.silenceJoystickConnectionWarning(true);
        flightStick = new CommandJoystick(port);
        flightThrottle = new CommandJoystick(port + 1);
        DPR = flightStick.povRight();
        DPD = flightStick.povDown();
        DPL = flightStick.povLeft();
        DPU = flightStick.povUp();
        RT = flightStick.button(0); // TODO: NEED TO FIND THE PORT OF BUTTON
    }

    public void bind(final Subsystems subsystems) {
        var shooter = subsystems.shooter;

        // Manual relative control command
        shooter.setDefaultCommand(
                ShooterCommands.manualRelativeControl(
                        shooter,
                        () -> -deadband(flightStick.getZ(), 0.1) * 80.0, // Turret: 50 deg/s
                        () -> -deadband(flightStick.getX(), 0.1) * 40.0, // Hood: 20 deg/s
                        () ->
                                -deadband(flightThrottle.getX(), 0.1)
                                        * 200.0)); // Flywheel: 200 RPM/s

        this.RT.whileTrue(
                IndexerCommands.goToState(subsystems.indexer, IndexerState.DISPENSE_BALL));
    }

    private double deadband(double val, double deadband) {
        if (Math.abs(val) > deadband) {
            if (val > 0.0) {
                return (val - deadband) / (1.0 - deadband);
            } else {
                return (val + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public DoubleSupplier getHorizontalAxisFlightStick() {
        return flightStick::getY; // this is flipped because on a controller the y axis is horizontal
    }

    public DoubleSupplier getVerticalAxisFlightStick() {
        return flightStick::getX; 
    }

    public DoubleSupplier getTwistAxisFlightStick() {
        return flightStick::getZ;
    }


}
