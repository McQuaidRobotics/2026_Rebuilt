package igknighters.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
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
        RT = flightStick.button(1);
    }

    public void bind(final Subsystems subsystems) {
        var shooter = subsystems.shooter;

        // Manual relative control command
        shooter.setDefaultCommand(
                ShooterCommands.manualRelativeControl(
                        shooter,
                        () ->
                                (-deadband(flightStick.getRawAxis(0), 0.1) * 120.0
                                        + -deadband(flightStick.getRawAxis(4), .1)
                                                * 20), // Turret: 50 deg/s
                        () -> getAngleDegrees(), // Hood: 20 deg/s
                        () ->
                                Math.max(0, deadband(-flightThrottle.getX() + 1.0, 0.02))
                                        * 3000.0)); // Flywheel: 200 RPM/s

        this.RT.whileTrue(
                IndexerCommands.goToState(subsystems.indexer, IndexerState.DISPENSE_BALL));
    }

    private double getAngleDegrees() {
        return ((-flightThrottle.getY() + 1.0)
                        / 2.0
                        * (Robot.consts.shooter().kHood().MAX_ANGLE_DEGREES()
                                - Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES())
                + Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());
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
        return flightStick
                ::getY; // this is flipped because on a controller the y axis is horizontal
    }

    public DoubleSupplier getVerticalAxisFlightStick() {
        return flightStick::getX;
    }

    public DoubleSupplier getTwistAxisFlightStick() {
        return flightStick::getZ;
    }
}
