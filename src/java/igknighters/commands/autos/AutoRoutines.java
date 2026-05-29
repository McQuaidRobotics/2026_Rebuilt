package igknighters.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.IndexerCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.commands.SwerveCommands;
import igknighters.constants.RobotConsts;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.YamShooter.ShooterState;
import igknighters.subsystems.YamsIntake.YamIntakeState;
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

    public AutoRoutine ORBIT_LEFT() {
        AutoRoutine routine = autoFactory.newRoutine("Orbit Left");
        AutoTrajectory swipe1Out = routine.trajectory("ORBIT_LEFT_1.traj");
        AutoTrajectory swipe1In = routine.trajectory("ORBIT_LEFT_2.traj");
        AutoTrajectory loopDiDoop = routine.trajectory("ORBIT_LEFT_3.traj");

        routine.active().onTrue(Commands.sequence(swipe1Out.resetOdometry(), swipe1Out.cmd()));

        swipe1Out.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        swipe1Out.done().onTrue(swipe1In.spawnCmd());

        swipe1In.done()
                .onTrue(
                        HigherOrderCommands.shootTillEmpty(subsystems, 4)
                                .andThen(loopDiDoop.spawnCmd()));

        loopDiDoop.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        loopDiDoop
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.shootTillEmpty(subsystems, 6)));

        return routine;
    }

    public AutoRoutine TEST() {
        AutoRoutine routine = autoFactory.newRoutine("Test Routine");

        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(
                                Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond()
                                        * 0.1)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

        routine.active()
                .onTrue(
                        Commands.sequence(
                                ShooterCommands.targetState(
                                                subsystems.shooter,
                                                new ShooterState(
                                                        RPM.of(500),
                                                        Degrees.of(270),
                                                        Degrees.of(
                                                                Robot.consts
                                                                        .shooter()
                                                                        .kHood()
                                                                        .MAX_ANGLE_DEGREES())))
                                        .withTimeout(2),
                                ShooterCommands.targetState(
                                                subsystems.shooter,
                                                new ShooterState(
                                                        RPM.of(2000),
                                                        Degrees.of(-90),
                                                        Degrees.of(
                                                                Robot.consts
                                                                        .shooter()
                                                                        .kHood()
                                                                        .MIN_ANGLE_DEGREES())))
                                        .withTimeout(2),
                                // shooter tested
                                subsystems
                                        .intake
                                        .targetState(YamIntakeState.DEPLOYED)
                                        .withTimeout(4.0),
                                // feed ball here
                                Commands.parallel(
                                                ShooterCommands.targetState(
                                                        subsystems.shooter,
                                                        new ShooterState(
                                                                RPM.of(500),
                                                                Degrees.of(0),
                                                                Degrees.of(
                                                                        Robot.consts
                                                                                .shooter()
                                                                                .kHood()
                                                                                .MIN_ANGLE_DEGREES()))),
                                                IndexerCommands.dispense(subsystems.indexer))
                                        .withTimeout(3),
                                // test feed and shot
                                Commands.print("MOVING IN 5"),
                                Commands.waitSeconds(1),
                                Commands.print("MOVING IN 4"),
                                Commands.waitSeconds(1),
                                Commands.print("MOVING IN 3"),
                                Commands.waitSeconds(1),
                                Commands.print("MOVING IN 2"),
                                Commands.waitSeconds(1),
                                Commands.print("MOVING IN 1"),
                                Commands.waitSeconds(1),
                                Commands.print("MOVING"),
                                swerve.run(
                                                () ->
                                                        subsystems.swerve.setControl(
                                                                m_driveRequest.withRotationalRate(
                                                                        RotationsPerSecond.of(.5))))
                                        .withTimeout(1.0),
                                swerve.run(
                                                () ->
                                                        subsystems.swerve.setControl(
                                                                m_driveRequest.withRotationalRate(
                                                                        RotationsPerSecond.of(
                                                                                -.5))))
                                        .withTimeout(1.0)
                                // spin back + forth one half rotation

                                ));
        return routine;
    }

    public AutoRoutine OP_LEFT() {
        AutoRoutine routine = autoFactory.newRoutine("OP LEFT");

        AutoTrajectory firstLoop = routine.trajectory("OP_LEFT_1.traj");
        AutoTrajectory transitionToSecondLoop = routine.trajectory("OP_LEFT_2.traj");
        AutoTrajectory secondLoop = routine.trajectory("OP_LEFT_3.traj");

        routine.active().onTrue(Commands.sequence(firstLoop.resetOdometry(), firstLoop.spawnCmd()));

        firstLoop.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        firstLoop
                .atTime("PROTECT")
                .onTrue(subsystems.intake.targetState(YamIntakeState.PARTIAL_STOW));

        firstLoop
                .done()
                .onTrue(
                        Commands.parallel(
                                HigherOrderCommands.shootTillEmpty(subsystems, 4.0),
                                transitionToSecondLoop.cmd()));

        transitionToSecondLoop.done().onTrue(secondLoop.spawnCmd());

        secondLoop.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        secondLoop
                .atTime("PROTECT_INTAKE_2")
                .onTrue(subsystems.intake.targetState(YamIntakeState.PARTIAL_STOW));
        secondLoop.atTime("SHOOT_2").onTrue(HigherOrderCommands.shootTillEmpty(subsystems, 5));
        secondLoop.done().onTrue(HigherOrderCommands.shootTillEmpty(subsystems, 10));

        return routine;
    }

    public AutoRoutine OP_RIGHT() {
        AutoRoutine routine = autoFactory.newRoutine("OP RIGHT");

        AutoTrajectory firstLoop = routine.trajectory("OP_RIGHT_1.traj");
        AutoTrajectory transitionToSecondLoop = routine.trajectory("OP_RIGHT_2.traj");
        AutoTrajectory secondLoop = routine.trajectory("OP_RIGHT_3.traj");

        routine.active().onTrue(Commands.sequence(firstLoop.resetOdometry(), firstLoop.spawnCmd()));

        firstLoop.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        firstLoop
                .atTime("PROTECT")
                .onTrue(subsystems.intake.targetState(YamIntakeState.PARTIAL_STOW));
        firstLoop
                .done()
                .onTrue(
                        Commands.parallel(
                                HigherOrderCommands.shootTillEmpty(subsystems, 4.0),
                                transitionToSecondLoop.cmd()));

        transitionToSecondLoop.done().onTrue(secondLoop.cmd());

        secondLoop.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        secondLoop
                .atTime("Protect_intake")
                .onTrue(subsystems.intake.targetState(YamIntakeState.PARTIAL_STOW));
        secondLoop.done().onTrue(HigherOrderCommands.shootTillEmpty(subsystems, 10));

        return routine;
    }

    public AutoRoutine BUMP_PASS_TO_SELF_LEFT() {
        AutoRoutine routine = autoFactory.newRoutine("Bump Pass to Self Left");

        AutoTrajectory trajectory = routine.trajectory("BUMP_PASS_TO_SELF_LEFT_1.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                trajectory.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 3),
                                trajectory.spawnCmd()));

        trajectory.atTime("HIPPO").onTrue(HigherOrderCommands.hippoShoot(subsystems));

        trajectory
                .atTime("JUST INTAKE")
                .onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        trajectory
                .atTime("PROTECT INTAKE")
                .onTrue(subsystems.intake.targetState(YamIntakeState.PARTIAL_STOW));
        trajectory
                .atTime("START SHOOTING AGAIN")
                .onTrue(HigherOrderCommands.hippoShoot(subsystems));
        trajectory
                .atTime("NO MUNCH HIPPO")
                .onTrue(
                        HigherOrderCommands.rapidFireStream(subsystems)
                                .alongWith(subsystems.intake.targetState(YamIntakeState.DEPLOYED)));

        trajectory
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.shootTillEmpty(subsystems, 15)));
        return routine;
    }

    public AutoRoutine ORBIT_PASS_TO_SELF_RIGHT() {
        AutoRoutine routine = autoFactory.newRoutine("Orbit Pass to Self Right");
        AutoTrajectory trajectory = routine.trajectory("PASS_TO_SELF_BUMP_1.traj");

        routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

        trajectory.atTime("HIPPO").onTrue(HigherOrderCommands.hippoShoot(subsystems));

        trajectory.atTime("IDLE").onTrue(HigherOrderCommands.IdleShooter(subsystems));

        trajectory.atTime("HIPPO_2").onTrue(HigherOrderCommands.hippoShoot(subsystems));
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
        meanTrajectory
                .atTime("INTAKE")
                .onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));
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

        intakeTrajectory.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

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

        intakeTrajectory
                .active()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    subsystems.swerve.setActiveTrajectory(intakeTrajectory);
                                }));

        routine.active()
                .onTrue(
                        Commands.sequence(
                                intakeTrajectory.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                Commands.parallel(
                                        subsystems.intake.targetState(YamIntakeState.DEPLOYED),
                                        intakeTrajectory.cmd())));

        intakeTrajectory
                .done()
                .onTrue(
                        Commands.sequence(
                                Commands.runOnce(
                                        () -> {
                                            subsystems.swerve.clearActiveTrajectory();
                                        }),
                                Commands.runOnce(
                                        () -> {
                                            subsystems.swerve.setActiveTrajectory(
                                                    scoringTrajectory);
                                        }),
                                scoringTrajectory.cmd()));

        scoringTrajectory.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));

        return routine;
    }

    public AutoRoutine orbitRight() {
        AutoRoutine routine = autoFactory.newRoutine("Orbit Right");

        AutoTrajectory swipe1Out = routine.trajectory("ORBIT_RIGHT_1.traj");
        AutoTrajectory swipe1In = routine.trajectory("ORBIT_RIGHT_2.traj");
        AutoTrajectory swipe2Loop = routine.trajectory("ORBIT_RIGHT_3.traj");
        routine.active().onTrue(Commands.sequence(swipe1Out.resetOdometry(), swipe1Out.cmd()));

        swipe1Out.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));
        swipe1Out.done().onTrue(swipe1In.cmd());

        swipe1In.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        swipe1In.done()
                .onTrue(
                        Commands.sequence(
                                SwerveCommands.stopDriving(swerve),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                Commands.parallel(
                                        swipe2Loop.cmd(),
                                        subsystems.intake.targetState(YamIntakeState.DEPLOYED),
                                        HigherOrderCommands.IdleShooter(subsystems))));

        swipe2Loop
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.shootTillEmpty(subsystems, 9)));

        return routine;
    }

    public AutoRoutine passToSelfLeft() {
        AutoRoutine routine = autoFactory.newRoutine("Pass to Self Left");
        AutoTrajectory PASS_TRAJECTORY = routine.trajectory("PASS_TO_SELF_LEFT_1.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                PASS_TRAJECTORY.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 3),
                                Commands.parallel(
                                        HigherOrderCommands.hippoShoot(subsystems),
                                        PASS_TRAJECTORY.cmd())));

        PASS_TRAJECTORY.active().onTrue(Commands.print("STARTING PASS TRAJECTORY"));

        PASS_TRAJECTORY
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.shootTillEmpty(subsystems, 10)));

        // PASS_TRAJECTORY.atTime("STOW").onTrue(HigherOrderCommands.rapidFireStream(subsystems));
        // PASS_TRAJECTORY.atTime("INTAKE").onTrue(HigherOrderCommands.hippoShoot(subsystems));

        return routine;
    }

    public AutoRoutine centerPreload() {
        AutoRoutine routine = autoFactory.newRoutine("Center Preload");
        AutoTrajectory move_traj = routine.trajectory("CENTER_SIMPLE.traj");

        routine.active().onTrue(Commands.sequence(move_traj.resetOdometry(), move_traj.cmd()));

        move_traj.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

        move_traj
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.shootTillEmpty(subsystems, 15)));
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

        tangential_traj.active().onTrue(subsystems.intake.targetState(YamIntakeState.DEPLOYED));

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
                                        subsystems.intake.targetState(YamIntakeState.DEPLOYED)),
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
                                                subsystems.intake.targetState(
                                                        YamIntakeState.DEPLOYED),
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
                                                                                                        .MIN_ANGLE_DEGREES())),
                                                                subsystems.shooter.hood,
                                                                subsystems.shooter.flywheels,
                                                                subsystems.shooter.turret),
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
