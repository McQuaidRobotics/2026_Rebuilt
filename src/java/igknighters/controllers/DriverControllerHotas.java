package igknighters.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.IndexerCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.commands.SwerveCommands;
import igknighters.commands.Wayfinder;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.IntakeState;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriverControllerHotas extends Controller {

    // Define the bindings for the controller

    // Define the buttons on the controller

    private final CommandJoystick flightStick;
    private final CommandJoystick throttleStick;

    private boolean intakeActive = false;

    /** Button: 1 */
    protected final Trigger ARM;

    protected final Trigger PINKY_LEVER;

    protected final Trigger HONEUP;

    protected final Trigger HONEDOWN;

    protected final Trigger HONELEFT;

    protected final Trigger HONERIGHT;

    protected final Trigger DUCAL_CLAW_PRESS;

    protected final Trigger SIDE_BUTTON;

    protected final Trigger PINKY_BUTTON;

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
    public DriverControllerHotas(int port) {
        DriverStation.silenceJoystickConnectionWarning(true);
        flightStick = new CommandJoystick(port);
        throttleStick = new CommandJoystick(port + 1);

        RT = flightStick.button(1);
        ARM = flightStick.button(2);
        SIDE_BUTTON = flightStick.button(3);
        DUCAL_CLAW_PRESS = flightStick.button(4);
        PINKY_BUTTON = flightStick.button(5);
        PINKY_LEVER = flightStick.button(6);
        HONEUP = flightStick.button(7);
        HONERIGHT = flightStick.button(8);
        HONEDOWN = flightStick.button(9);
        HONELEFT = flightStick.button(10);

        DPR = flightStick.povRight();
        DPD = flightStick.povDown();
        DPL = flightStick.povLeft();
        DPU = flightStick.povUp();
    }

    public Supplier<Pose2d> poseSupplier(Subsystems subsystems) {
        return () -> subsystems.swerve.getState().Pose;
    }

    public void bind(final Subsystems subsystems) {
        var swerve = subsystems.swerve;
        var intake = subsystems.intake;

        this.PINKY_LEVER.whileTrue(IntakeCommands.holdAtIntake(subsystems.intake));
        this.RT
                .whileTrue(HigherOrderCommands.rapidFireStream(subsystems))
                .onFalse(HigherOrderCommands.IdleShooter(subsystems));
        this.DPR.whileTrue(IndexerCommands.unBlock(subsystems.indexer));
        this.ARM.whileTrue(HigherOrderCommands.forceDispense(subsystems));
        this.PINKY_BUTTON.whileTrue(IntakeCommands.intakeWhileSlightJorking(intake));
        this.SIDE_BUTTON.onTrue(SwerveCommands.zeroGyro(swerve));
        this.HONEUP.whileTrue(IntakeCommands.expell(subsystems.intake));
        this.HONEDOWN.onTrue(IntakeCommands.holdAtState(subsystems.intake, IntakeState.FULL_STOW));
        this.DPD.whileTrue(ShooterCommands.homeHood(subsystems.shooter));
        this.DPR.and(this.HONELEFT).whileTrue(Wayfinder.driveToSafeSpot(swerve));
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
    public DoubleSupplier getTranslationX() {
        return () -> flightStick.getRawAxis(0);
    }

    /**
     * Right on the stick is positive (axis 4)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the right stick x axis
     */
    public DoubleSupplier translationX(double deadband) {
        return deadbandSupplier(getTranslationX(), deadband);
    }

    /**
     * Up on the stick is positive (axis 5)
     *
     * @return A supplier for the value of the right stick y axis
     */
    public DoubleSupplier getTranslationY() {
        return () -> -flightStick.getRawAxis(1);
    }

    /**
     * Up on the stick is positive (axis 5)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the right stick y axis
     */
    public DoubleSupplier translationY(double deadband) {
        return deadbandSupplier(getTranslationY(), deadband);
    }

    public DoubleSupplier getThrottle() {
        return () -> (throttleStick.getRawAxis(0) - 1.0) / -2.0; // keep within [0, 1]
    }

    @Override
    public DoubleSupplier getShotModifier() {
        return () -> (1 + throttleStick.getRawAxis(2));
    }

    /**
     * Right on the stick is positive (axis 0)
     *
     * @return A supplier for the value of the left stick x axis
     */
    public DoubleSupplier getRotationX() {
        return () -> -flightStick.getRawAxis(4);
    }

    public DoubleSupplier getRotationY() {
        return () -> 0.0;
    }

    /**
     * Right on the stick is positive (axis 0)
     *
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the left stick x axis
     */
    public DoubleSupplier leftStickX(double deadband) {
        return deadbandSupplier(getRotationX(), deadband);
    }

    /**
     * Will rumble both sides of the controller with a magnitude
     *
     * @param magnitude The magnitude to rumble at
     */
    public void rumble(double magnitude) {
        flightStick.getHID().setRumble(RumbleType.kBothRumble, magnitude);
    }
}
