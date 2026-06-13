package igknighters.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.IndexerCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.commands.SwerveCommands;
import igknighters.constants.RobotConsts;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.IntakeState;
import igknighters.subsystems.shooter.ShooterState;

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

    public AutoRoutine PASS_TO_SELF_RIGHT_WITH_DEPOT_AND_HUMAN_PLAYER() {
        AutoRoutine routine =
                autoFactory.newRoutine("Pass to Self Right with Depot and Human Player");
        AutoTrajectory trajectory = routine.trajectory("DO_IT_MYSELF.traj");

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

    public AutoRoutine OP_RIGHT() {
        return OP("RIGHT");
    }

    public AutoRoutine OP_LEFT() {
        return OP("LEFT");
    }

    public AutoRoutine SQUOVAL_RIGHT() {
        return SQUOVAL("RIGHT");
    }

    public AutoRoutine SQUOVAL_LEFT() {
        return SQUOVAL("LEFT");
    }

    public AutoRoutine ORBIT_RIGHT() {
        return orbit("RIGHT");
    }

    public AutoRoutine ORBIT_LEFT() {
        return orbit("LEFT");
    }

    public AutoRoutine BUMP_PASS_TO_SELF_LEFT() {
        return bump_pass_to_self("LEFT");
    }

    public AutoRoutine BUMP_PASS_TO_SELF_RIGHT() {
        return bump_pass_to_self("RIGHT");
    }

    public AutoRoutine MEAN_LEFT() {
        return meanRoutine("LEFT");
    }

    public AutoRoutine MEAN_RIGHT() {
        return meanRoutine("RIGHT");
    }

    public AutoRoutine SINGLE_SWIPE_LEFT() {
        return SINGLE_DUMP("LEFT");
    }

    public AutoRoutine SINGLE_SWIPE_RIGHT() {
        return SINGLE_DUMP("RIGHT");
    }

    public AutoRoutine HIPPO_LEFT() {
        return hippo("LEFT");
    }

    public AutoRoutine SINGLE_SWIPE_DEPOT() {
        AutoRoutine routine = autoFactory.newRoutine("SINGLE SWIPE DEPOT");
        AutoTrajectory gobbleTraj = routine.trajectory("SINGLE_SWIPE_DEPO_1.traj");
        AutoTrajectory shootTraj = routine.trajectory("SINGLE_SWIPE_DEPO_2.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                gobbleTraj.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                gobbleTraj.spawnCmd()));

        gobbleTraj.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        gobbleTraj.done().onTrue(shootTraj.spawnCmd());

        shootTraj.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));
        return routine;
    }

    public AutoRoutine SINGLE_SWIPE_OUTPOST() {
        AutoRoutine routine = autoFactory.newRoutine("SINGLE SWIPE OUTPOST");
        AutoTrajectory gobbleTraj = routine.trajectory("SINGLE_SWIPE_OUTPOST_1.traj");
        AutoTrajectory shootTraj = routine.trajectory("SINGLE_SWIPE_OUTPOST_2.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                gobbleTraj.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                gobbleTraj.spawnCmd()));

        gobbleTraj.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        gobbleTraj.done().onTrue(shootTraj.spawnCmd());

        shootTraj.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));
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
                                IntakeCommands.holdAtIntake(subsystems.intake).withTimeout(4.0),
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

    private AutoRoutine SINGLE_DUMP(String direction) {
        AutoRoutine routine = autoFactory.newRoutine("SINGLE_DUMP " + direction);

        AutoTrajectory firstLoop = routine.trajectory("SINGLE_DUMP_" + direction + ".traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                firstLoop.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                firstLoop.spawnCmd()));

        firstLoop.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        firstLoop
                .atTime("PROTECT")
                .onTrue(IntakeCommands.holdAtState(subsystems.intake, IntakeState.partialStow));

        firstLoop
                .done()
                .onTrue(Commands.parallel(HigherOrderCommands.shootTillEmpty(subsystems, 25.0)));

        return routine;
    }

    private AutoRoutine SQUOVAL(String direction) {
        AutoRoutine routine = autoFactory.newRoutine("SQUOVAL " + direction);
        AutoTrajectory trajectory = routine.trajectory("SQUOVAL_" + direction + "_1.traj");
        AutoTrajectory pass = routine.trajectory("SQUOVAL_" + direction + "_2.traj");

        routine.active()
                .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.spawnCmd()));

        trajectory.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        trajectory
                .atTime("PROTECT")
                .onTrue(IntakeCommands.holdAtState(subsystems.intake, IntakeState.partialStow));

        trajectory.atTime("SHOOT").onTrue(HigherOrderCommands.shootTillEmpty(subsystems, 7));

        trajectory.done().onTrue(pass.spawnCmd());

        pass.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));
        return routine;
    }

    private AutoRoutine OP(String direction) {
        AutoRoutine routine = autoFactory.newRoutine("OP " + direction);

        AutoTrajectory firstLoop = routine.trajectory("OP_" + direction + "_1.traj");
        AutoTrajectory transitionToSecondLoop = routine.trajectory("OP_" + direction + "_2.traj");
        AutoTrajectory secondLoop = routine.trajectory("OP_" + direction + "_3.traj");

        routine.active().onTrue(Commands.sequence(firstLoop.resetOdometry(), firstLoop.spawnCmd()));

        firstLoop.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        firstLoop
                .atTime("PROTECT")
                .onTrue(IntakeCommands.holdAtState(subsystems.intake, IntakeState.partialStow));

        firstLoop
                .done()
                .onTrue(
                        Commands.parallel(
                                HigherOrderCommands.shootTillEmpty(subsystems, 4.0),
                                transitionToSecondLoop.cmd()));

        transitionToSecondLoop.done().onTrue(secondLoop.cmd());

        secondLoop.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        secondLoop
                .atTime("Protect_intake")
                .onTrue(IntakeCommands.holdAtState(subsystems.intake, IntakeState.partialStow));

        secondLoop.done().onTrue(HigherOrderCommands.shootTillEmpty(subsystems, 10));

        return routine;
    }

    private AutoRoutine bump_pass_to_self(String direction) {
        AutoRoutine routine = autoFactory.newRoutine("Orbit Pass to Self " + direction);
        AutoTrajectory trajectory =
                routine.trajectory("PASS_TO_SELF_BUMP_" + direction + "_1.traj");

        routine.active()
                .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.spawnCmd()));

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
    private AutoRoutine meanRoutine(String direction) {
        AutoRoutine routine = autoFactory.newRoutine("Mean Routine");
        AutoTrajectory meanTrajectory = routine.trajectory("MEAN_AUTO_" + direction + "_1.traj");

        routine.active()
                .onTrue(
                        Commands.sequence(
                                meanTrajectory.resetOdometry(),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                meanTrajectory.spawnCmd()));
        // steal balls from them we shouldnt get too many bc intake backwards but any balls shot is
        // better then none
        meanTrajectory.active().onTrue(HigherOrderCommands.hippoShoot(subsystems));
        // start preserving balls for shots instead of just stealing to our side
        meanTrajectory.atTime("INTAKE").onTrue(IntakeCommands.holdAtIntake(subsystems.intake));
        // back on our side so shoot gathered balls + human player station
        meanTrajectory.atTime("SCORE").onTrue(HigherOrderCommands.hippoShoot(subsystems));
        return routine;
    }

    private AutoRoutine orbit(String direction) {
        AutoRoutine routine = autoFactory.newRoutine("Orbit " + direction);

        AutoTrajectory swipe1Out = routine.trajectory("ORBIT_" + direction + "_1.traj");
        AutoTrajectory swipe1In = routine.trajectory("ORBIT_" + direction + "_2.traj");
        AutoTrajectory swipe2Loop = routine.trajectory("ORBIT_" + direction + "_3.traj");
        routine.active().onTrue(Commands.sequence(swipe1Out.resetOdometry(), swipe1Out.cmd()));

        swipe1Out.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));
        swipe1Out.done().onTrue(swipe1In.cmd());

        swipe1In.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

        swipe1In.done()
                .onTrue(
                        Commands.sequence(
                                SwerveCommands.stopDriving(swerve),
                                HigherOrderCommands.shootTillEmpty(subsystems, 5),
                                Commands.parallel(
                                        swipe2Loop.cmd(),
                                        IntakeCommands.holdAtIntake(subsystems.intake),
                                        HigherOrderCommands.IdleShooter(subsystems))));

        swipe2Loop
                .done()
                .onTrue(
                        SwerveCommands.stopDriving(swerve)
                                .andThen(HigherOrderCommands.shootTillEmpty(subsystems, 9)));

        return routine;
    }

    public AutoRoutine centerPreload() {
        AutoRoutine routine = autoFactory.newRoutine("Center Preload");
        AutoTrajectory move_traj = routine.trajectory("CENTER_SIMPLE.traj");

        routine.active().onTrue(Commands.sequence(move_traj.resetOdometry(), move_traj.cmd()));

        move_traj.active().onTrue(IntakeCommands.holdAtIntake(subsystems.intake));

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

    private AutoRoutine hippo(String direction) {
        AutoRoutine routine = autoFactory.newRoutine(direction + "Neutral Hippo");

        AutoTrajectory moveTraj = routine.trajectory("NEUTRAL_HIPPO_" + direction + "_1.traj");

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
                                                moveTraj.spawnCmd()))
                                .withName(direction + "NEUTRAL HIPPO"));
        moveTraj.atTimeBeforeEnd(0.0).onTrue(SwerveCommands.stopDriving(swerve));
        return routine;
    }
}
