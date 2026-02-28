package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.teleop.AutoRotateOnBump;
import igknighters.constants.AbleToShootSharedState;
import igknighters.constants.Conv;
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

    public Pose2d getPoseFromString(String path) {
        double x = dashboardTable.getEntry(path + "X").getDouble(0.0) * Conv.FEET_TO_METERS;
        double y = dashboardTable.getEntry(path + "Y").getDouble(0.0) * Conv.FEET_TO_METERS;
        double theta = dashboardTable.getEntry(path + "Theta").getDouble(0.0);
        return new Pose2d(x, y, new Rotation2d(theta));
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
                ShooterCommands.shootAt(
                        shooter,
                        () -> swerve.getState().Pose,
                        swerve::getFieldRelativeSpeeds,
                        () -> getPoseFromString("robot/passWaypoint")));
        moveToTrigger.whileTrue(
                Repulsor.moveWithRepulsor(swerve, getPoseFromString("robot/moveWaypoint"), 1));
    }

    public void SetupTriggers(Subsystems subsystems, DriverController driverController) {
        Led led = subsystems.led;
        Swerve swerve = subsystems.swerve;
        Trigger onBump = new Trigger(() -> FieldConstants.BUMP.isInside(swerve.getState().Pose));

        SetupOperatorController(subsystems);

        onBump.whileTrue(new AutoRotateOnBump(swerve, driverController));

        falseOnce()
                .and(disabled)
                .whileTrue(
                        LEDCommands.run(led, LEDPattern.solid(Color.kRed))
                                .ignoringDisable(true)
                                .withName("DisabledRed"));

        autonomous.whileTrue(
                LEDCommands.run(led, LedUtil.makeRainbow(255, 128))
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
                .canShootTrigger()
                .and(ableToShootState.beingControlledTrigger())
                .whileTrue(LEDCommands.run(led, LEDPattern.solid(Color.kYellow)));
        ableToShootState
                .canShootTrigger()
                .and(ableToShootState.beingControlledTrigger().negate())
                .whileFalse(LEDCommands.run(led, LEDPattern.solid(Color.kPurple)));
    }
}
