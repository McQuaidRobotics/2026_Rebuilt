package igknighters.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

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
import igknighters.commands.SwerveCommands;
import igknighters.constants.FieldConstants;
import igknighters.constants.RobotConsts;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;

public class AutoRoutines extends AutoCommands {

    RobotConsts consts;

    public AutoRoutines(Subsystems subsystems, AutoFactory factory, RobotConsts consts) {
        super(subsystems, factory);
        this.consts = consts;

        if (Robot.isSimulation()) {
            new Trigger(DriverStation::isAutonomousEnabled)
                    .onTrue(
                            Commands.waitSeconds(20.0)
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
            return robotPose2d.getY() > FieldConstants.Y_FIELD / 2
                    ? FieldConstants.PASS.POSITION_LEFT_BLUE
                    : FieldConstants.PASS.POSITION_RIGHT_BLUE;
        } else {
            Pose2d robotPose2d = subsystems.swerve.getState().Pose;
            return robotPose2d.getY() > FieldConstants.Y_FIELD / 2
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
                                moveTraj.resetOdometry(),
                                Commands.print("ODOMETRY RESET"),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                Commands.print("ALL BALLS SHOT"),
                                moveTraj.cmd()));

        moveTraj.atTime("Intake")
                .onTrue(
                        Commands.sequence(
                                Commands.print("INTAKE BALLS"),
                                HigherOrderCommands.hippoShoot(subsystems)));

        return routine;
    }

    //     public AutoRoutine rightOutpostClimb() {
    //         AutoRoutine routine = autoFactory.newRoutine("Right Outpost Climb");
    //         AutoTrajectory outpostTraj = routine.trajectory("RIGHT_OUTPOST_CLIMB.traj");

    //         routine.active()
    //                 .onTrue(
    //                         Commands.sequence(
    //                                         outpostTraj.resetOdometry(),
    //                                         HigherOrderCommands.shootTillEmpty(subsystems, 3),
    //                                         Commands.parallel(
    //                                                 HigherOrderCommands.hippoShoot(subsystems),
    //                                                 outpostTraj.cmd()))
    //                                 .withName("Right Outpost Climb"));
    //         outpostTraj
    //                 .done()
    //                 .onTrue(
    //                         SwerveCommands.stopDriving(swerve)
    //
    // .andThen(HigherOrderCommands.prepToClimbFirstRung(subsystems)));

    //         return routine;
    //     }

    //     public AutoRoutine leftOutpostClimb() {
    //         AutoRoutine routine = autoFactory.newRoutine("Left Outpost Climb");
    //         AutoTrajectory outpostTraj = routine.trajectory("LEFT_OUTPOST_CLIMB.traj");

    //         routine.active()
    //                 .onTrue(
    //                         Commands.sequence(
    //                                         outpostTraj.resetOdometry(),
    //                                         HigherOrderCommands.shootTillEmpty(subsystems, 3),
    //                                         Commands.parallel(
    //
    // IntakeCommands.holdAtIntake(subsystems.intake),
    //
    // HigherOrderCommands.rapidFireStream(subsystems),
    //                                                 outpostTraj.cmd()))
    //                                 .withName("Left Outpost Climb"));
    //         outpostTraj
    //                 .done()
    //                 .onTrue(
    //                         SwerveCommands.stopDriving(swerve)
    //
    // .andThen(HigherOrderCommands.prepToClimbFirstRung(subsystems)));

    //         return routine;
    //     }

    //     public AutoRoutine leftDepoClimb() {
    //         AutoRoutine routine = autoFactory.newRoutine("Left Depo Climb");
    //         AutoTrajectory depoTraj = routine.trajectory("LEFT_DEPO_CLIMB.traj"); // test
    //         routine.active()
    //                 .onTrue(
    //                         Commands.sequence(
    //                                         depoTraj.resetOdometry(),
    //                                         HigherOrderCommands.shootTillEmpty(subsystems, 3),
    //                                         Commands.parallel(
    //
    // IntakeCommands.holdAtIntake(subsystems.intake),
    //
    // HigherOrderCommands.rapidFireStream(subsystems),
    //                                                 depoTraj.cmd()))
    //                                 .withName("Left Depo Climb"));
    //         depoTraj.done()
    //                 .onTrue(
    //                         SwerveCommands.stopDriving(swerve)
    //
    // .andThen(HigherOrderCommands.prepToClimbFirstRung(subsystems)));
    //         return routine;
    //     }

    //     public AutoRoutine rightDepoClimb() {
    //         AutoRoutine routine = autoFactory.newRoutine("Right Depo Climb");
    //         AutoTrajectory depoTraj = routine.trajectory("RIGHT_DEPO_CLIMB.traj");

    //         routine.active()
    //                 .onTrue(
    //                         Commands.sequence(
    //                                         depoTraj.resetOdometry(),
    //                                         HigherOrderCommands.shootTillEmpty(subsystems, 3),
    //                                         Commands.parallel(
    //                                                 HigherOrderCommands.hippoShoot(subsystems),
    //                                                 depoTraj.cmd()))
    //                                 .withName("Right Depo Climb"));
    //         depoTraj.done()
    //                 .onTrue(
    //                         SwerveCommands.stopDriving(swerve)
    //
    // .andThen(HigherOrderCommands.prepToClimbFirstRung(subsystems)));

    //         return routine;
    //     }

    public AutoRoutine rightNuetralHippo() {
        AutoRoutine routine = autoFactory.newRoutine("Right Neutral Hippo");

        AutoTrajectory moveTraj = routine.trajectory("RIGHT_NEUTRAL_HIPPO.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                        moveTraj.resetOdometry(),
                                        HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                        Commands.parallel(
                                                IntakeCommands.holdAtIntake(subsystems.intake),
                                                Commands.sequence(
                                                        Commands.runOnce(
                                                                () ->
                                                                        subsystems.shooter
                                                                                .targetState(
                                                                                        RPM.of(
                                                                                                2000),
                                                                                        Degrees.of(
                                                                                                0.0),
                                                                                        Degrees.of(
                                                                                                Robot
                                                                                                        .consts
                                                                                                        .shooter()
                                                                                                        .kHood()
                                                                                                        .MIN_ANGLE_DEGREES()))),
                                                        Commands.waitSeconds(1.0),
                                                        HigherOrderCommands.rapidFireStream(
                                                                subsystems)),
                                                moveTraj.cmd()))
                                .withName("RIGHT NUETRAL HIPPO"));
        moveTraj.atTimeBeforeEnd(0.0).onTrue(SwerveCommands.stopDriving(swerve));
        return routine;
    }

    public AutoRoutine PASS_TO_SELF_RIGHT_WITH_DEPOT_AND_HUMAN_PLAYER() {
        AutoRoutine routine =
                autoFactory.newRoutine("Pass to Self Right with Depot and Human Player");
        AutoTrajectory trajectory = routine.trajectory("PASS_TO_SELF_RIGHT_1.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                trajectory.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 3),
                                Commands.parallel(
                                        HigherOrderCommands.hippoShoot(subsystems),
                                        trajectory.cmd())));

        trajectory.done().onTrue(SwerveCommands.stopDriving(swerve));

        return routine;
    }

    /**
     * LEFT -> Right Human player station Designed to colide in center line with the back of robot
     * facing other teams intake Will pass the first half and then will gather balls untill enter
     * trench Once under trench intake + score going to human player station
     */
    public AutoRoutine meanRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("Mean Routine");
        AutoTrajectory meanTrajectory = routine.trajectory("MEAN_AUTO_1.traj");

        routine.active().onTrue(meanTrajectory.resetOdometry().andThen(meanTrajectory.cmd()));
        // steal balls from them we shouldnt get too many bc intake backwards but any balls shot is
        // better then none
        meanTrajectory.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));
        // start preserving balls for shots instead of just stealing to our side
        meanTrajectory.atTime("INTAKE").onTrue(IntakeCommands.holdAtIntake(subsystems.intake));
        // back on our side so shoot gathered balls + human player station
        meanTrajectory.atTime("SCORE").onTrue(HigherOrderCommands.hippoShoot(subsystems));
        return routine;
    }

    public AutoRoutine singleSwipeLeft() {
        AutoRoutine routine = autoFactory.newRoutine("Single Swipe Left");

        AutoTrajectory intakeTrajectory = routine.trajectory("LEFT_SINGLE_SWIPE_1.traj");
        AutoTrajectory scoringTrajectory = routine.trajectory("LEFT_SINGLE_SWIPE_2.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                intakeTrajectory.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 4),
                                intakeTrajectory.cmd()));

        intakeTrajectory.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        intakeTrajectory.done().onTrue(scoringTrajectory.cmd());

        scoringTrajectory.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));

        scoringTrajectory
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .alongWith(HigherOrderCommands.hippoShoot(subsystems)));

        return routine;
    }

    public AutoRoutine singleSwipeRight() {
        AutoRoutine routine = autoFactory.newRoutine("Single Swipe Right");

        AutoTrajectory intakeTrajectory = routine.trajectory("RIGHT_SINGLE_SWIPE_1.traj");
        AutoTrajectory scoringTrajectory = routine.trajectory("RIGHT_SINGLE_SWIPE_2.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                intakeTrajectory.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                Commands.parallel(
                                        IntakeCommands.holdAtIntake(subsystems.intake),
                                        intakeTrajectory.cmd())));

        intakeTrajectory.done().onTrue(scoringTrajectory.cmd());

        scoringTrajectory.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));

        return routine;
    }

    public AutoRoutine orbitRight() {
        AutoRoutine routine = autoFactory.newRoutine("Orbit Right");

        AutoTrajectory swipe1Out = routine.trajectory("ORBIT_RIGHT_1.traj");
        AutoTrajectory swipe1In = routine.trajectory("ORBIT_RIGHT_2.traj");
        AutoTrajectory swipe2Out = routine.trajectory("ORBIT_RIGHT_3.traj");
        AutoTrajectory swipe2In = routine.trajectory("ORBIT_RIGHT_4.traj");
        routine.active().onTrue(Commands.sequence(swipe1Out.resetOdometry(), swipe1Out.cmd()));

        swipe1Out.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));
        swipe1Out.done().onTrue(swipe1In.cmd());

        swipe1In.active().onTrue(IntakeCommands.holdAtStow(subsystems.intake));

        swipe1In.done()
                .onTrue(
                        Commands.sequence(
                                SwerveCommands.stopDriving(swerve),
                                HigherOrderCommands.shootTillEmpty(subsystems, 4),
                                Commands.parallel(
                                        swipe2Out.cmd(),
                                        IntakeCommands.holdAtIntake(subsystems.intake),
                                        HigherOrderCommands.IdleShooter(subsystems))));

        swipe2Out.done().onTrue(SwerveCommands.stopDriving(swerve).andThen(swipe2In.cmd()));

        swipe2In.done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.rapidFireStream(subsystems)));

        return routine;
    }

    public AutoRoutine passToSelfLeft() {
        AutoRoutine routine = autoFactory.newRoutine("Pass to Self Left");
        AutoTrajectory PASS_TRAJECTORY = routine.trajectory("PASS_TO_SELF_LEFT_1.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                PASS_TRAJECTORY.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                Commands.parallel(
                                        HigherOrderCommands.hippoShoot(subsystems),
                                        PASS_TRAJECTORY.cmd())));

        PASS_TRAJECTORY.active().onTrue(Commands.print("STARTING PASS TRAJECTORY"));
        PASS_TRAJECTORY
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.hippoShoot(subsystems)));

        // PASS_TRAJECTORY.atTime("STOW").onTrue(HigherOrderCommands.rapidFireStream(subsystems));
        // PASS_TRAJECTORY.atTime("INTAKE").onTrue(HigherOrderCommands.hippoShoot(subsystems));

        return routine;
    }

    public AutoRoutine centerPreload() {
        AutoRoutine routine = autoFactory.newRoutine("Center Preload");
        AutoTrajectory move_traj = routine.trajectory("CENTER_SIMPLE.traj");

        routine.active().onTrue(Commands.sequence(move_traj.resetOdometry(), move_traj.cmd()));

        move_traj.atTime("SHOOT").onTrue(HigherOrderCommands.hippoShoot(subsystems));

        move_traj.done().onTrue(SwerveCommands.stopDriving(swerve));
        return routine;
    }

    public AutoRoutine SOTMTEST() {
        AutoRoutine routine = autoFactory.newRoutine("SOTM TEST");
        AutoTrajectory tangential_traj = routine.trajectory("SOTM_TEST_1.traj");
        AutoTrajectory reset_traj = routine.trajectory("SOTM_TEST_2.traj");
        AutoTrajectory radial_away_traj = routine.trajectory("SOTM_TEST_3.traj");
        AutoTrajectory radial_towards_traj = routine.trajectory("SOTM_TEST_4.traj");
        routine.active()
                .onTrue(
                        Commands.sequence(
                                tangential_traj.resetOdometry(),
                                Commands.waitSeconds(5),
                                Commands.parallel(tangential_traj.cmd())));

        tangential_traj.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        tangential_traj.atTime("SHOOT").onTrue(HigherOrderCommands.hippoShoot(subsystems));

        tangential_traj
                .done()
                .onTrue(
                        Commands.sequence(
                                SwerveCommands.stopDriving(swerve),
                                Commands.parallel(
                                        reset_traj.cmd(),
                                        HigherOrderCommands.IdleShooter(subsystems))));

        reset_traj
                .done()
                .onTrue(
                        Commands.sequence(
                                SwerveCommands.stopDriving(swerve),
                                Commands.deadline(
                                        Commands.waitSeconds(5),
                                        IntakeCommands.holdAtIntake(subsystems.intake)),
                                Commands.parallel(
                                        HigherOrderCommands.hippoShoot(subsystems),
                                        radial_away_traj.cmd())));

        radial_away_traj
                .done()
                .onTrue(
                        Commands.sequence(
                                SwerveCommands.stopDriving(swerve),
                                Commands.parallel(
                                        radial_towards_traj.cmd(),
                                        HigherOrderCommands.hippoShoot(subsystems))));

        radial_towards_traj.done().onTrue(SwerveCommands.stopDriving(swerve));
        return routine;
    }

    public AutoRoutine leftNuetralHippo() {
        AutoRoutine routine = autoFactory.newRoutine("New Left Neutral Hippo");

        AutoTrajectory moveTraj = routine.trajectory("LEFT_NEUTRAL_HIPPO_1.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                        moveTraj.resetOdometry(),
                                        HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                        Commands.print("FINISHED EMPTYING HOPPER"),
                                        Commands.parallel(
                                                IntakeCommands.holdAtIntake(subsystems.intake),
                                                Commands.sequence(
                                                        subsystems.shooter.runOnce(
                                                                () ->
                                                                        subsystems.shooter
                                                                                .targetState(
                                                                                        RPM.of(
                                                                                                2000),
                                                                                        Degrees.of(
                                                                                                0.0),
                                                                                        Degrees.of(
                                                                                                Robot
                                                                                                        .consts
                                                                                                        .shooter()
                                                                                                        .kHood()
                                                                                                        .MIN_ANGLE_DEGREES()))),
                                                        Commands.waitSeconds(1.0),
                                                        HigherOrderCommands.rapidFireStream(
                                                                subsystems)),
                                                moveTraj.cmd()))
                                .withName("LEFT NUETRAL HIPPO"));
        moveTraj.atTimeBeforeEnd(0.0).onTrue(SwerveCommands.stopDriving(swerve));
        return routine;
    }

    public AutoRoutine centerOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("Center Outpost climb");

        AutoTrajectory moveTraj = routine.trajectory("CENTER_OUTPOST_CLIMB.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                        moveTraj.resetOdometry(),
                                        HigherOrderCommands.shootTillEmpty(subsystems, 2),
                                        Commands.parallel(
                                                HigherOrderCommands.rapidFireStream(subsystems),
                                                moveTraj.cmd()))
                                .withName("Move and Shoot"));
        moveTraj.atTimeBeforeEnd(0.0).onTrue(SwerveCommands.stopDriving(swerve));
        return routine;
    }

    //     public AutoRoutine centerDepotClimb() {
    //         AutoRoutine routine = autoFactory.newRoutine("Center Depot climb");

    //         AutoTrajectory moveTraj = routine.trajectory("CENTER_DEPOT_CLIMB.traj");

    //         routine.active()
    //                 .onTrue(
    //                         Commands.sequence(
    //                                 moveTraj.resetOdometry(),
    //                                 Commands.print("ODOMETRY RESET"),
    //                                 moveTraj.cmd()
    //                                         .alongWith(
    //                                                 HigherOrderCommands.hippoShoot(subsystems)
    //                                                         .repeatedly(),
    //                                                 Commands.print("IM PARELELING").repeatedly())
    //                                         .withName("Intake and Shoot")));

    //         moveTraj.done()
    //                 .onTrue(
    //                         Commands.sequence(
    //                                 SwerveCommands.stopDriving(swerve),
    //                                 HigherOrderCommands.prepToClimbFirstRung(subsystems)));
    //         return routine;
    //     }
}
