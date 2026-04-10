package igknighters.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.IndexerCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.Shooter.AimingCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.commands.SwerveCommands;
import igknighters.commands.Wayfinder;
import igknighters.constants.DrivingSharedState;
import igknighters.subsystems.Subsystems;
import java.util.function.DoubleSupplier;
import vroom.LiveFollower;

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
        DrivingSharedState state = DrivingSharedState.getInstance();
        var swerve = subsystems.swerve;
        var shooter = subsystems.shooter;
        var indexer = subsystems.indexer;

        if (debugType == DebugType.SWERVE) {
            this.A.whileTrue(Wayfinder.driveToTarget(swerve, new Pose2d(0, 0, new Rotation2d(0))));
            this.B.whileTrue(LiveFollower.driveLive(swerve, new Pose2d(8, 5, new Rotation2d(0))));
        } else if (debugType == DebugType.SHOOTER) {
            this.A.whileTrue(ShooterCommands.targetState(shooter, 5000, 90, 25));
            this.B.whileTrue(ShooterCommands.targetState(shooter, 5000, 180, 30));
            this.X.whileTrue(ShooterCommands.targetState(shooter, 5000, 270, 35));
            this.Y.whileTrue(ShooterCommands.targetState(shooter, 5000, 360, 40));
            // this.RT.whileTrue(IndexerCommands.dispense(indexer));
            // this.LT.whileTrue(IndexerCommands.stopDispensing(indexer));
            this.RT.whileTrue(AimingCommands.shootWithProtection(subsystems.shooter));
            this.LT.whileTrue(IndexerCommands.dispense(indexer));

            // this.DPD.whileTrue(ShooterCommands.targetState(shooter, 0, 0,
            // kHood.MAX_ANGLE_DEGREES));
            // this.DPR.whileTrue(ShooterCommands.targetNetworkTablesValues(shooter));
            // this.DPU.whileTrue(ShooterCommands.targetState(shooter, 0, 0,
            // kHood.MIN_ANGLE_DEGREES));

        } else if (debugType == DebugType.INDEXER) {
            this.A.onTrue(IndexerCommands.dispense(indexer));
            this.B.onTrue(IndexerCommands.justStop(indexer));
        } else if (debugType == DebugType.INTAKE) {
            this.A.whileTrue(IntakeCommands.holdAtIntake(subsystems.intake));
            this.B.whileTrue(IntakeCommands.jorkIt(subsystems.intake));
            this.X.whileTrue(IntakeCommands.slightJorkIntake(subsystems.intake));
            this.Y.onTrue(IntakeCommands.toggleHoldState(subsystems.intake));
        } else {
            System.out.println("UNKNOWN DEBUG TYPE: " + debugType);
            throw new IllegalArgumentException("UNKNOWN DEBUG TYPE: " + debugType);
        }
    }

    public void bind(final Subsystems subsystems) {
        var swerve = subsystems.swerve;
        var intake = subsystems.intake;

        this.LT
                .whileTrue(IntakeCommands.holdAtIntake(subsystems.intake))
                .onFalse(IntakeCommands.holdAtStow(subsystems.intake));
        this.RT
                .whileTrue(HigherOrderCommands.rapidFireStream(subsystems))
                .onFalse(HigherOrderCommands.IdleShooter(subsystems));
        this.DPR.whileTrue(IndexerCommands.unBlock(subsystems.indexer));
        this.RB.whileTrue(HigherOrderCommands.forceDispense(subsystems));
        this.LB.whileTrue(IntakeCommands.intakeWhileSlightJorking(intake));
        this.Start.onTrue(SwerveCommands.zeroGyro(swerve));
        this.X.whileTrue(IntakeCommands.expell(subsystems.intake));
        this.DPD.whileTrue(ShooterCommands.homeHood(subsystems.shooter));
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
