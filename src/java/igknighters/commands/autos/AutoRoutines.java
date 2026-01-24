package igknighters.commands.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.IntakeCommands;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;

public class AutoRoutines extends AutoCommands {

    public AutoRoutines(Subsystems subsystems, AutoFactory factory) {
        super(subsystems, factory);

        if (Robot.isSimulation()) {
            new Trigger(DriverStation::isAutonomousEnabled)
                    .onTrue(
                            Commands.waitSeconds(15.3)
                                    .andThen(() -> DriverStationSim.setEnabled(false))
                                    .withName("Simulated Auto Ender"));
        }
    }

    public Supplier<Command> trajTest(String trajName) {
        return () ->
                Commands.sequence(
                        autoFactory.resetOdometry(trajName), autoFactory.trajectoryCmd(trajName));
    }

    @FunctionalInterface
    public interface DualSideAuto {
        Command generate();
    }

    public static void addCmd(AutoChooser chooser, String name, DualSideAuto auto) {
        chooser.addCmd(name, () -> auto.generate());
    }

    // public Command driveAround() {
    //     return newAuto("shoot_then_pass")
    //             .addDrivingTrajectory(
    //                     Waypoints.STARTING_RIGHT, Waypoints.BUMP_LAND_RIGHT,
    // Waypoints.BALLS_RIGHT)
    //             .build();
    // }

    public Supplier<Command> shootThenMove() {
        return () ->
                newRebuiltAuto("SHOOT-THEN-MOVE")
                        .shootThenMove(Waypoints.STARTING_RIGHT, Waypoints.BUMP_LAND_RIGHT, 5.0)
                        .addDrivingTrajectory(Waypoints.BUMP_LAND_RIGHT, Waypoints.BALLS_RIGHT)
                        .shootAndMove(Waypoints.BALLS_RIGHT, Waypoints.BALLS_MIDDLE)
                        .build();
    }

    public Command rightToLeft() {
        return newRebuiltAuto("right to left")
                .shootAndMove(Waypoints.STARTING_RIGHT, Waypoints.BUMP_LAND_RIGHT)
                .shootAndMove(Waypoints.BALLS_RIGHT, Waypoints.BALLS_MIDDLE)
                .build();
    }

    public Pose3d getHubTarget() {
        return Robot.isBlue() ? FieldConstants.HUB.POSE3D_BLUE : FieldConstants.HUB.POSE3D_RED;
    }

    public Pose3d getPassTarget() {
        if (Robot.isBlue()) {
            Pose2d robotPose2d = subsystems.swerve.getState().Pose;
            return robotPose2d.getY() > FieldConstants.WIDTH / 2
                    ? FieldConstants.PASS.POSITION_LEFT_BLUE
                    : FieldConstants.PASS.POSITION_RIGHT_BLUE;
        } else {
            Pose2d robotPose2d = subsystems.swerve.getState().Pose;
            return robotPose2d.getY() > FieldConstants.WIDTH / 2
                    ? FieldConstants.PASS.POSITION_LEFT_RED
                    : FieldConstants.PASS.POSITION_RIGHT_RED;
        }
    }

    public AutoRoutine scoreThenPass() {
        AutoRoutine routine = autoFactory.newRoutine("Score then pass");

        AutoTrajectory moveTraj = routine.trajectory("ShootThenIntake.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                        moveTraj.resetOdometry().withTimeout(0.1),
                                        HigherOrderCommands.shootTillEmpty(
                                                        subsystems, 3.0, () -> getHubTarget())
                                                .withName("SCORE_THEN_PASS_Shoot"),
                                        Commands.print("FINISHED SCORING"),
                                        Commands.waitSeconds(3),
                                        moveTraj.cmd().withName("SCORE_THEN_PASS_Move"))
                                .withName("SCORE THEN MOVE ELEMENT"));

        moveTraj.atTime("Intake")
                .onTrue(
                        Commands.sequence(
                                Commands.print("INTAKE BALLS"),
                                Commands.parallel(
                                        IntakeCommands.intakeBalls(subsystems.intake),
                                        HigherOrderCommands.shootNoStop(
                                                subsystems, () -> getHubTarget()))));

        return routine;
    }

    public AutoRoutine leftNeutralHippo() {
        AutoRoutine routine = autoFactory.newRoutine("Left Neutral Hippo");

        AutoTrajectory moveTraj = routine.trajectory("LeftNeutralHippo.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                        moveTraj.resetOdometry().withTimeout(0.1),
                                        HigherOrderCommands.shootTillEmpty(
                                                        subsystems, 3.0, () -> getHubTarget())
                                                .withName("LEFT_NEUTRAL_HIPPO_Shoot"),
                                        Commands.print("FINISHED SCORING"),
                                        Commands.waitSeconds(1),
                                        moveTraj.cmd().withName("LEFT_NEUTRAL_HIPPO_Move"))
                                .withName("LEFT NEUTRAL HIPPO MOVE ELEMENT"));

        moveTraj.atTime("Start Intaking")
                .onTrue(
                        Commands.sequence(
                                Commands.print("INTAKE BALLS"),
                                Commands.parallel(
                                        IntakeCommands.intakeBalls(subsystems.intake),
                                        HigherOrderCommands.shootNoStop(
                                                subsystems, () -> getHubTarget()))));
        moveTraj.atTime("Intake and Pass.")
                .onTrue(
                        Commands.sequence(
                                Commands.print("INTAKE BALLS"),
                                Commands.parallel(
                                        IntakeCommands.intakeBalls(subsystems.intake),
                                        HigherOrderCommands.shootNoStop(
                                                subsystems, () -> getHubTarget()))));                                        
        return routine;
    }
}
