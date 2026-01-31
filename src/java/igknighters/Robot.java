// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.IndexerCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.DrivingSharedState;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import java.util.Optional;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private AutoFactory autoFactory;
    public final AutoChooser autoChooser = new AutoChooser();

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final SubsystemTriggers subsystemTriggers = new SubsystemTriggers();

    private final DriverController driverController = new DriverController(0);

    public final Subsystems subsytems;

    private final boolean kUseLimelight = true;

    private final SwerveConsts swerveConstGetter = new SwerveConsts();

    private final CommonSwerveConsts swerveConsts = swerveConstGetter.getSwerveConsts();

    private Telemetry logger;
    TunableDouble detune = TunableValues.getDouble("Tunables/Detune", 0.6);
    TunableDouble targetingP = TunableValues.getDouble("Tunables/TargetingP", 0.07);
    TunableDouble targetingI = TunableValues.getDouble("Tunables/TargetingI", 0.00);
    TunableDouble targetingD = TunableValues.getDouble("Tunables/TargetingD", 0.00);

    public void setUpCommandLogging() {
        scheduler.onCommandInitialize(
                command ->
                        DogLog.log(
                                "Commands/Tracking/" + command.getName() + "/ Command Running",
                                "TRUE"));

        scheduler.onCommandInitialize(
                command ->
                        DogLog.log(
                                "Commands/Tracking/" + command.getName() + "/ Command Interrupted",
                                "FALSE"));

        scheduler.onCommandInterrupt(
                command ->
                        DogLog.log(
                                "Commands/Tracking/" + command.getName() + "/ Command Interrupted",
                                "TRUE"));
        scheduler.onCommandFinish(
                command ->
                        DogLog.log(
                                "Commands/Tracking/" + command.getName() + "/ Command Running",
                                "FALSE"));
        scheduler.onCommandFinish(
                command ->
                        DogLog.log(
                                "Commands/Tracking/" + command.getName() + "/ Command Interrupted",
                                "FALSE"));
    }

    public void publishCommandsAndSubystems(Subsystems subsystems) {
        SmartDashboard.putData(CommandScheduler.getInstance());
        for (var subsystem : subsystems.lockedResources) {
            SmartDashboard.putData(subsystem);
        }
    }

    public void setUpAutos(Subsystems subsystems) {
        autoFactory = subsytems.swerve.createAutoFactory();
        final var routines = new AutoRoutines(subsytems, autoFactory);
        autoChooser.addCmd("shoot-then-move", routines.shootThenMove());
        autoChooser.addCmd("TRAJECTORY TEST", routines.trajTest("Straight"));
        autoChooser.addRoutine(
                "NEW METHOD IDK IF THIS WILL WORK HOPEFULLY IT WILL", routines::scoreThenPass);
        autoChooser.addRoutine("NEW LEFT NUETRAL HIPPO", routines::newLeftNuetralHippo);
        autoChooser.addRoutine("CENTER OUTPOST CLIMB", routines::centerOutpostClimb);
        autoChooser.addRoutine("Center Depot climb", routines::centerDepotClimb);
        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
    }

    public void setUpSwerve(Subsystems subsystems) {
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, 1.0));

        logger = new Telemetry(swerveConsts.getMaxSpeedMetersPerSecond(), subsytems);
        subsytems.swerve.registerTelemetry(logger::telemeterize);
    }

    public void setUpTest(Subsystems subsystems) {
        SmartDashboard.putData(
                "Commands/Spindexer/Spindexer - STOP", IndexerCommands.);
        SmartDashboard.putData(
                "Commands/Spindexer/Spindexer - DISPENSE BALLS",
                IndexerCommands.dispense(subsystems.indexer));
    }

    public Robot() {
        setUpCommandLogging();
        subsytems =
                new Subsystems(
                        swerveConsts.createDrivetrain(),
                        new LimeLightVision(),
                        new Led(40, 1),
                        new Shooter(),
                        new Indexer(),
                        new Intake(),
                        new Climber());
        setUpSwerve(subsytems);
        publishCommandsAndSubystems(subsytems);
        setUpAutos(subsytems);
        setUpTest(subsytems);
        bindDriverController();

        subsystemTriggers.SetupTriggers(subsytems.led);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (kUseLimelight) {
            var driveState = subsytems.swerve.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            Pose2d currentPose =
                    subsytems.vision.getRobotPoseFromVision(headingDeg, omegaRps, 0, 0, 0, 0);
            if (currentPose != null) {
                subsytems.swerve.addVisionMeasurement(
                        currentPose, subsytems.vision.getLastTimeStamp());
            }
        }
    }

    public void bindDriverController() {
        driverController.bind(subsytems);
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
        scheduler.getActiveButtonLoop().clear();
        // CommandScheduler.getInstance().clearComposedCommands();
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, detune.value()));
        DrivingSharedState.getInstance().setDetune(detune.value());
        DrivingSharedState.getInstance().setKP(targetingP.value());
        DrivingSharedState.getInstance().setKI(targetingI.value());
        DrivingSharedState.getInstance().setKD(targetingD.value());

        bindDriverController();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Command autoCommand = autoChooser.selectedCommand();
        scheduler.schedule(autoCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        scheduler.cancelAll();
    }

    @Override
    public void teleopInit() {
        scheduler.cancelAll();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        // for (var subsystem : subsytems.locklessResources) {
        //     subsystem.simulationPeriodic();
        // }
    }

    public static boolean isBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent()) {
            return (ally.get() == Alliance.Blue);
        } else {
            return true; // Default to blue if alliance is unknown
        }
    }
}
