package igknighters.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.IndexerCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.indexer.IndexerState;
import java.util.function.DoubleSupplier;

public class OperatorController {

    private final CommandXboxController controller;

    /** Button: 1 */
    protected final Trigger A;

    /** Button: 2 */
    protected final Trigger B;

    /** Button: 3 */
    protected final Trigger X;

    /** Button: 4 */
    protected final Trigger Y;

    /** Left Center; Button: 7 */
    protected final Trigger Back;

    /** Right Center; Button: 8 */
    protected final Trigger Start;

    /** Left Bumper; Button: 5 */
    protected final Trigger LB;

    /** Right Bumper; Button: 6 */
    protected final Trigger RB;

    /** Left Stick; Button: 9 */
    protected final Trigger LS;

    /** Right Stick; Button: 10 */
    protected final Trigger RS;

    /** Left Trigger; Axis: 2 */
    protected final Trigger LT;

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
        controller = new CommandXboxController(port);
        A = controller.a();
        B = controller.b();
        X = controller.x();
        Y = controller.y();
        LB = controller.leftBumper();
        RB = controller.rightBumper();
        Back = controller.back();
        Start = controller.start();
        LS = controller.leftStick();
        RS = controller.rightStick();
        LT = controller.leftTrigger(0.25);
        RT = controller.rightTrigger(0.25);
        DPR = controller.povRight();
        DPD = controller.povDown();
        DPL = controller.povLeft();
        DPU = controller.povUp();
    }

    public void bind(final Subsystems subsystems) {
        var shooter = subsystems.shooter;

        // Manual relative control command
        shooter.setDefaultCommand(
                ShooterCommands.manualRelativeControl(
                        shooter,
                        () -> -deadband(controller.getLeftX(), 0.1) * 80.0, // Turret: 50 deg/s
                        () -> -deadband(controller.getLeftY(), 0.1) * 40.0, // Hood: 20 deg/s
                        () ->
                                -deadband(controller.getRightY(), 0.1)
                                        * 200.0)); // Flywheel: 100 RPM/s

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

    public DoubleSupplier leftStickX() {
        return controller::getLeftX;
    }

    public DoubleSupplier leftStickY() {
        return () -> -controller.getLeftY();
    }
}
