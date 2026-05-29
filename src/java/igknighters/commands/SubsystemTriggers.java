package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.LEDCommands.LEDSection;
import igknighters.commands.Shooter.AimingCommands;
import igknighters.commands.teleop.AutoRotateOnBump;
import igknighters.commands.teleop.SlowedDownDrivingWhileShooting;
import igknighters.constants.Conv;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.constants.ShootInformation;
import igknighters.controllers.DriverController;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.YamShooter.Shooter.shotType;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SubsystemTriggers {
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger autonomous = RobotModeTriggers.autonomous();
    private final Trigger teleop = RobotModeTriggers.teleop();

    private Trigger shouldRumble;
    private final NetworkTable dashboardTable =
            NetworkTableInstance.getDefault().getTable("dashboard");

    Command disabledLED;

    Command autoLED;

    Command teleopLED;

    public static Trigger falseOnce() {
        return new Trigger(
                new BooleanSupplier() {
                    boolean ret = false;

                    public boolean getAsBoolean() {
                        try {
                            return ret;
                        } finally {
                            ret = true;
                        }
                    }
                    ;
                });
    }

    public static Command runLEDBASEDONSTATE(Led led, boolean isDisabled, boolean isAutonomous) {
        // should be Magenta, Cyan, Yellow if all three are lit then it should shoot

        if (isDisabled) {
            return LEDCommands.run(led, LEDPattern.solid(Color.kRed));
        } else if (isAutonomous) {
            LEDSection possibleShot;
            LEDSection atTarget;
            LEDSection beingControlled;
            LEDSection autonomous =
                    new LEDSection(1, 0, LedUtil.makeRainbow(255, 128), 20, "AUTONOMOUS RAINBOW");

            if (ShootInformation.getInstance().shotPosible().getAsBoolean()) {
                possibleShot =
                        new LEDSection(
                                0, 0, LEDPattern.solid(Color.kMagenta), 20, "IS POSSIBLE SHOT");
            } else {
                possibleShot =
                        new LEDSection(
                                0, 0, LEDPattern.solid(Color.kBlack), 20, "IS NOT POSSIBLE SHOT");
            }

            if (ShootInformation.getInstance().getAtTarget()) {
                atTarget = new LEDSection(0, 20, LEDPattern.solid(Color.kCyan), 20, "AT TARGET");
            } else {
                atTarget =
                        new LEDSection(0, 20, LEDPattern.solid(Color.kBlack), 20, "NOT AT TARGET");
            }

            if (ShootInformation.getInstance().beingControlledTrigger().getAsBoolean()) {
                beingControlled =
                        new LEDSection(
                                0, 40, LEDPattern.solid(Color.kYellow), 20, "BEING CONTROLLED");
            } else {
                beingControlled =
                        new LEDSection(
                                0, 40, LEDPattern.solid(Color.kBlack), 20, "NOT BEING CONTROLLED");
            }

            return LEDCommands.run(led, autonomous, possibleShot, atTarget, beingControlled);
        } else {
            LEDSection possibleShot;
            LEDSection atTarget;
            LEDSection beingControlled;
            LEDSection enabled =
                    new LEDSection(1, 0, LEDPattern.solid(Color.kGreen), 40, "ENABLED");
            if (ShootInformation.getInstance().getAtTarget()) {
                atTarget = new LEDSection(0, 20, LEDPattern.solid(Color.kCyan), 20, "AT TARGET");
            } else {
                atTarget =
                        new LEDSection(0, 20, LEDPattern.solid(Color.kBlack), 20, "NOT AT TARGET");
            }
            if (ShootInformation.getInstance().beingControlledTrigger().getAsBoolean()) {
                beingControlled =
                        new LEDSection(
                                0, 40, LEDPattern.solid(Color.kYellow), 20, "BEING CONTROLLED");
            } else {
                beingControlled =
                        new LEDSection(
                                0, 40, LEDPattern.solid(Color.kBlack), 20, "NOT BEING CONTROLLED");
            }
            if (ShootInformation.getInstance().shotPosible().getAsBoolean()) {
                possibleShot =
                        new LEDSection(
                                0, 0, LEDPattern.solid(Color.kMagenta), 20, "IS POSSIBLE SHOT");
            } else {
                possibleShot =
                        new LEDSection(
                                0, 0, LEDPattern.solid(Color.kBlack), 20, "IS NOT POSSIBLE SHOT");
            }

            return LEDCommands.run(led, enabled, possibleShot, atTarget, beingControlled);
        }
    }

    public Pose3d getPoseFromString(String path) {
        double x = dashboardTable.getEntry(path + "X").getDouble(0.0) * Conv.FEET_TO_METERS;
        double y = dashboardTable.getEntry(path + "Y").getDouble(0.0) * Conv.FEET_TO_METERS;
        double theta = dashboardTable.getEntry(path + "Theta").getDouble(0.0);
        return new Pose3d(x, y, 0, new Rotation3d(0, 0, theta));
    }

    public void SetupOperatorController(Subsystems subsystems) {
        Swerve swerve = subsystems.swerve;

        Trigger moveToTrigger =
                new Trigger(() -> dashboardTable.getEntry("robot/moveTrigger").getBoolean(false));
        Trigger passTrigger =
                new Trigger(() -> dashboardTable.getEntry("robot/passTrigger").getBoolean(false));

        passTrigger.onTrue(
                Commands.runOnce(
                        () -> ShootInformation.getInstance().useOperatorControlLocation(true)));
        passTrigger.onFalse(
                Commands.runOnce(
                        () -> ShootInformation.getInstance().useOperatorControlLocation(false)));
        moveToTrigger.whileTrue(
                Repulsor.moveWithRepulsor(
                        swerve, getPoseFromString("robot/moveWaypoint").toPose2d()));
    }

    public Command getLEDCommandByMode(Led led) {
        return Commands.either(
                teleopLED(led), Commands.either(disabledLED(led), autoLED(led), disabled), teleop);
    }

    private Command teleopLED(Led led) {
        return LEDCommands.run(led, LEDPattern.solid(Color.kGreen))
                .ignoringDisable(true)
                .withName("TeleopGreen");
    }

    private Command autoLED(Led led) {
        return LEDCommands.run(led, LedUtil.makeRainbow(255, 126))
                .ignoringDisable(true)
                .withName("AutoRainbow");
    }

    private Command disabledLED(Led led) {
        return LEDCommands.run(led, LEDPattern.solid(Color.kRed))
                .ignoringDisable(true)
                .withName("DisabledRed");
    }

    public void SetupTriggers(
            Subsystems subsystems,
            DriverController driverController,
            Supplier<Pose2d> poseSupplier) {
        Led led = subsystems.led;
        Swerve swerve = subsystems.swerve;
        Trigger onBump = new Trigger(() -> FieldConstants.BUMP.isInside(swerve.getState().Pose));

        Trigger trenchProtection = new Trigger(() -> DrivingSharedState.getInstance().underTrench);

        SetupOperatorController(subsystems);

        // onBump.and(teleop)
        //        .whileTrue(
        //                Commands.runOnce(() -> DrivingSharedState.getInstance().setOnBump(true))
        //                        .andThen(new AutoRotateOnBump(swerve, driverController)));
        // onBump.onFalse(Commands.runOnce(() ->
        // DrivingSharedState.getInstance().setOnBump(false)));

        onBump.and(teleop)
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(
                                        () -> DrivingSharedState.getInstance().setOnBump(true)),
                                new AutoRotateOnBump(swerve, driverController)));
        onBump.onFalse(Commands.runOnce(() -> DrivingSharedState.getInstance().setOnBump(false)));

        falseOnce().and(disabled).whileTrue(disabledLED(led));

        autonomous.onTrue(autoLED(led));

        teleop.whileTrue(teleopLED(led));

        // Get the AbleToShootSharedState singleton
        ShootInformation ableToShootState = ShootInformation.getInstance();

        // Bind LED commands to the canShootTrigger
        trenchProtection
                .onTrue(LEDCommands.run(led, LEDPattern.solid(Color.kBlue)))
                .onFalse(getLEDCommandByMode(led));
        ableToShootState
                .canShoot()
                .whileTrue(LEDCommands.run(led, LEDPattern.solid(Color.kMagenta)))
                .onFalse(getLEDCommandByMode(led));

        ableToShootState
                .beingControlledTrigger()
                .and(teleop)
                .and(() -> AimingCommands.getShotType() == shotType.SHOT)
                .whileTrue(new SlowedDownDrivingWhileShooting(swerve, driverController));

        // rumble
        shouldRumble =
                new Trigger(() -> subsystems.vision.timeSinceLastSample() < 0.1)
                        .and(falseOnce())
                        .and(teleop)
                        .whileTrue(
                                Commands.startEnd(
                                                () -> driverController.rumble(.30),
                                                () -> driverController.rumble(0.0))
                                        .ignoringDisable(true)
                                        .withName("RumbleForTag"));
    }
}
