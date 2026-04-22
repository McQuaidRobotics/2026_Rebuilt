package igknighters.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.Wayfinder;
import igknighters.subsystems.Subsystems;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriverController {

    // Define the bindings for the controller

    // Define the buttons on the controller

    private final CommandXboxController controller;

    private boolean intakeActive = false;

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

    /** for button idx (nice for sim) {@link edu.wpi.first.wpilibj.XboxController.Button} */
    public DriverController(int port) {
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

    public static enum DebugType {
        SHOOTER,
        SWERVE,
        INTAKE,
        INDEXER,
        CLIMBER;
    }

    public void bind(final Subsystems subsystems, DebugType debugType) {
        // allows for individual debuging of systems
        var swerve = subsystems.swerve;

        if (debugType == DebugType.SWERVE) {
            this.X.whileTrue(Wayfinder.driveToTarget(swerve, new Pose2d(3, 1, new Rotation2d(0))));
        } else if (debugType == DebugType.SHOOTER) {
        } else if (debugType == DebugType.INDEXER) {

        } else if (debugType == DebugType.INTAKE) {

        } else {
            System.out.println("UNKNOWN DEBUG TYPE: " + debugType);
            throw new IllegalArgumentException("UNKNOWN DEBUG TYPE: " + debugType);
        }
    }

    public Supplier<Pose2d> poseSupplier(Subsystems subsystems) {
        return () -> subsystems.swerve.getState().Pose;
    }

    public void bind(final Subsystems subsystems) {
        // when you write your commands this will be where you place them
        // this.A.whileTrue(...) -> will run the command inside parenthesis only while held
        // this.A.onTrue(...) -> will run the command inside parenthesis when pressed and will keep
        // running if its a .run() command
        // I have absolute faith. You do not need to add any binds to swerve driving will work with
        // no additional code. Zeroing is also
        // unnecessary if you have vision. However if you want it
        // this.start.onTrue(SwerveCommands.zeroGyro(swerve));
    }

    private DoubleSupplier deadbandSupplier(DoubleSupplier supplier, double deadband) {

        return () -> {
            double val = supplier.getAsDouble();
            if (Math.abs(val) > deadband) {
                if (val > 0.0) {
                    val = (val - deadband) / (1.0 - deadband);
                } else {

                    val = (val + deadband) / (1.0 - deadband);
                }
            } else {
                val = 0.0;
            }
            return val;
        };
    }

    /**
     * Right on the stick is positive (axis 4)
     *
     * @return A supplier for the value of the right stick x axis
     */
    public DoubleSupplier rightStickX() {
        return () -> -controller.getRightX();
    }

    /**
     * Right on the stick is positive (axis 4)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the right stick x axis
     */
    public DoubleSupplier rightStickX(double deadband) {
        return deadbandSupplier(rightStickX(), deadband);
    }

    /**
     * Up on the stick is positive (axis 5)
     *
     * @return A supplier for the value of the right stick y axis
     */
    public DoubleSupplier rightStickY() {
        return controller::getRightY;
    }

    /**
     * Up on the stick is positive (axis 5)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the right stick y axis
     */
    public DoubleSupplier rightStickY(double deadband) {
        return deadbandSupplier(rightStickY(), deadband);
    }

    /**
     * Right on the stick is positive (axis 0)
     *
     * @return A supplier for the value of the left stick x axis
     */
    public DoubleSupplier leftStickX() {
        return controller::getLeftX;
    }

    /**
     * Right on the stick is positive (axis 0)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the left stick x axis
     */
    public DoubleSupplier leftStickX(double deadband) {
        return deadbandSupplier(leftStickX(), deadband);
    }

    /**
     * Up on the stick is positive (axis 1)
     *
     * @return A supplier for the value of the left stick y axis
     */
    public DoubleSupplier leftStickY() {
        return () -> -controller.getLeftY();
    }

    /**
     * Up on the stick is positive (axis 1)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the left stick y axis
     */
    public DoubleSupplier leftStickY(double deadband) {
        return deadbandSupplier(leftStickY(), deadband);
    }

    /**
     * will print warning if this trigger is also bound to a command
     *
     * @param suppressWarning if true will not print warning even if bound to a command
     */
    public DoubleSupplier rightTrigger(boolean suppressWarning) {
        return controller::getRightTriggerAxis;
    }

    /**
     * will print warning if this trigger is also bound to a command
     *
     * @param suppressWarning if true will not print warning even if bound to a command
     */
    public DoubleSupplier leftTrigger(boolean suppressWarning) {
        return controller::getLeftTriggerAxis;
    }

    /**
     * Will rumble both sides of the controller with a magnitude
     *
     * @param magnitude The magnitude to rumble at
     */
    public void rumble(double magnitude) {
        controller.getHID().setRumble(RumbleType.kBothRumble, magnitude);
    }
}
