package igknighters.commands;

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
import igknighters.commands.teleop.AutoRotateOnBump;
import igknighters.constants.AbleToShootSharedState;
import igknighters.constants.Conv;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.controllers.DriverController;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberState;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;

public class SubsystemTriggers {
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger autonomous = RobotModeTriggers.autonomous();
    private final Trigger teleop = RobotModeTriggers.teleop();
    private final NetworkTable dashboardTable =
            NetworkTableInstance.getDefault().getTable("dashboard");
    private boolean shotPossible = false;
    private boolean atState = false;
    private boolean beingCommanded = false;

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

            if (AbleToShootSharedState.getInstance().shotPosible().getAsBoolean()) {
                possibleShot =
                        new LEDSection(
                                0, 0, LEDPattern.solid(Color.kMagenta), 20, "IS POSSIBLE SHOT");
            } else {
                possibleShot =
                        new LEDSection(
                                0, 0, LEDPattern.solid(Color.kBlack), 20, "IS NOT POSSIBLE SHOT");
            }

            if (AbleToShootSharedState.getInstance().getAtTarget()) {
                atTarget = new LEDSection(0, 20, LEDPattern.solid(Color.kCyan), 20, "AT TARGET");
            } else {
                atTarget =
                        new LEDSection(0, 20, LEDPattern.solid(Color.kBlack), 20, "NOT AT TARGET");
            }

            if (AbleToShootSharedState.getInstance().beingControlledTrigger().getAsBoolean()) {
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
            if (AbleToShootSharedState.getInstance().getAtTarget()) {
                atTarget = new LEDSection(0, 20, LEDPattern.solid(Color.kCyan), 20, "AT TARGET");
            } else {
                atTarget =
                        new LEDSection(0, 20, LEDPattern.solid(Color.kBlack), 20, "NOT AT TARGET");
            }
            if (AbleToShootSharedState.getInstance().beingControlledTrigger().getAsBoolean()) {
                beingControlled =
                        new LEDSection(
                                0, 40, LEDPattern.solid(Color.kYellow), 20, "BEING CONTROLLED");
            } else {
                beingControlled =
                        new LEDSection(
                                0, 40, LEDPattern.solid(Color.kBlack), 20, "NOT BEING CONTROLLED");
            }
            if (AbleToShootSharedState.getInstance().shotPosible().getAsBoolean()) {
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
        Climber climber = subsystems.climber;
        Swerve swerve = subsystems.swerve;
        Shooter shooter = subsystems.shooter;

        Trigger prepClimb =
                new Trigger(() -> (dashboardTable.getEntry("climb/stage").getDouble(2) == 0));
        Trigger pullUpClimb =
                new Trigger(() -> (dashboardTable.getEntry("climb/stage").getDouble(2) == 1));
        Trigger stowClimbTrigger =
                new Trigger(() -> (dashboardTable.getEntry("climb/stage").getDouble(2) == 2));
        prepClimb.onTrue(ClimberCommands.goToState(climber, ClimberState.LATCH_ON));
        pullUpClimb.onTrue(ClimberCommands.goToState(climber, ClimberState.PULL_UP));
        stowClimbTrigger.onTrue(ClimberCommands.goToState(climber, ClimberState.STOW));

        Trigger moveToTrigger =
                new Trigger(() -> dashboardTable.getEntry("robot/moveTrigger").getBoolean(false));
        Trigger passTrigger =
                new Trigger(() -> dashboardTable.getEntry("robot/passTrigger").getBoolean(false));

        passTrigger.whileTrue(
                HigherOrderCommands.fireAtTarget(
                        subsystems, getPoseFromString("robot/passWaypoint")));
        moveToTrigger.whileTrue(
                Repulsor.moveWithRepulsor(
                        swerve, getPoseFromString("robot/moveWaypoint").toPose2d(), 1));
    }

    public void SetupTriggers(Subsystems subsystems, DriverController driverController) {
        Led led = subsystems.led;
        Swerve swerve = subsystems.swerve;
        Trigger onBump = new Trigger(() -> FieldConstants.BUMP.isInside(swerve.getState().Pose));

        SetupOperatorController(subsystems);

        onBump.whileTrue(
                Commands.runOnce(() -> DrivingSharedState.getInstance().setOnBump(true))
                        .andThen(new AutoRotateOnBump(swerve, driverController)));
        onBump.onFalse(Commands.runOnce(() -> DrivingSharedState.getInstance().setOnBump(false)));

        falseOnce()
                .and(disabled)
                .whileTrue(
                        LEDCommands.run(led, LEDPattern.solid(Color.kRed))
                                .ignoringDisable(true)
                                .withName("DisabledRed"));

        autonomous.whileTrue(
                LEDCommands.run(led, LedUtil.makeRainbow(255, 126))
                        .ignoringDisable(true)
                        .withName("AutoRainbow"));

        teleop.whileTrue(
                LEDCommands.run(led, LEDPattern.solid(Color.kGreen))
                        .ignoringDisable(true)
                        .withName("TeleopGreen"));

        // Get the AbleToShootSharedState singleton
        AbleToShootSharedState ableToShootState = AbleToShootSharedState.getInstance();

        // Bind LED commands to the canShootTrigger

        ableToShootState
                .canShoot()
                .whileTrue(LEDCommands.run(led, LedUtil.makeBounce(Color.kCyan, .3)));
        // ableToShootState
        //         .atCommandedStateTrigger()
        //         .and(ableToShootState.beingControlledTrigger().negate())
        //         .whileFalse(LEDCommands.run(led, LEDPattern.solid(Color.kPurple)));
    }
}
