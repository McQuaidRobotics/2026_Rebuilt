// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.commands.IndexerCommands;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.Conv;
import igknighters.constants.DrivingSharedState;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.FuelSim;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private AutoFactory autoFactory;
    public final AutoChooser autoChooser = new AutoChooser();
    double i = 0;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final SubsystemTriggers subsystemTriggers = new SubsystemTriggers();

    private final DriverController driverController = new DriverController(0);

    public final Subsystems subsytems;

    private FuelSim fuelSim;
    private double lastShotTime = 0.0;

    private final boolean kUseLimelight = true;

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
            SmartDashboard.putData("SubsystemCommands/" + subsystem.getName(), subsystem);
        }
    }

    public void setUpAutos(Subsystems subsystems) {
        autoFactory = subsytems.swerve.createAutoFactory();
        final var routines = new AutoRoutines(subsytems, autoFactory);
        autoChooser.addRoutine("LEFT NUETRAL HIPPO", routines::leftNuetralHippo);
        autoChooser.addRoutine("RIGHT NUETRAL HIPPO", routines::rightNuetralHippo);
        autoChooser.addRoutine("CENTER OUTPOST CLIMB", routines::centerOutpostClimb);
        autoChooser.addRoutine("Center Depot climb", routines::centerDepotClimb);
        autoChooser.addRoutine("Right Depo Climb", routines::rightDepoClimb);
        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
    }

    public void setUpSwerve(Subsystems subsystems) {
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, 1.0));

        logger = new Telemetry(subsytems.swerve.getMaxSpeedMetersPerSecond(), subsytems);
        subsytems.swerve.registerTelemetry(logger::telemeterize);
    }

    public void setUpTest(Subsystems subsystems) {
        SmartDashboard.putData(
                "Commands/Spindexer/Spindexer - STOP",
                IndexerCommands.stopDispensing(subsystems.indexer));
        SmartDashboard.putData(
                "Commands/Spindexer/Spindexer - DISPENSE BALLS",
                IndexerCommands.dispense(subsystems.indexer));
    }

    public void setUpAdvantageScope() {

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
        try {
            Logger.recordMetadata(
                    "Hostname",
                    InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
        } catch (UnknownHostException e) {
            Logger.recordMetadata("Hostname", "Unknown");
        }
        Logger.recordMetadata(
                "Platform",
                "%s %s (%s)"
                        .formatted(
                                System.getProperty("os.name").replace(" ", ""),
                                System.getProperty("os.version"),
                                System.getProperty("os.arch")));
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        // Set timing mode
        setUseTiming(true);

        // Start AdvantageKit logger
        Logger.start();
    }

    public Robot() {
        setUpAdvantageScope();
        setUpCommandLogging();
        subsytems =
                new Subsystems(
                        new Swerve(false),
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

        subsystemTriggers.SetupTriggers(subsytems, driverController);

        if (isSimulation()) {
            configureFuelSim();
        }
    }

    public Robot(boolean isSwerveDisabled) {
        setUpAdvantageScope();
        setUpCommandLogging();
        subsytems =
                new Subsystems(
                        new Swerve(isSwerveDisabled),
                        new LimeLightVision(),
                        new Led(80, 2),
                        new Shooter(),
                        new Indexer(),
                        new Intake(),
                        new Climber());
        setUpSwerve(subsytems);
        publishCommandsAndSubystems(subsytems);
        setUpAutos(subsytems);
        setUpTest(subsytems);
        bindDriverController();

        subsystemTriggers.SetupTriggers(subsytems, driverController);
    }

    public Pose3d getTurretPose(double turretAngleDegrees) {
        // Assuming the turret is mounted at the center of the robot and has a fixed height
        double xMeterOffset = -0.1; // X offset from robot center to turret
        double yMeterOffset = -0.12; // Y offset from robot center to turret
        double zMeterOffset = 0.3; // Height of the turret from the ground
        return new Pose3d(
                xMeterOffset,
                yMeterOffset,
                zMeterOffset,
                new Rotation3d(0, 0, turretAngleDegrees * Math.PI / 180));
    }

    public Pose3d getHoodPose(double hoodAngleDegrees) {
        double dx = 0.09; // X offset from turret center to hood
        double dy = 0.0; // Y offset from turret center to hood
        double dz = 0.12; // z offset from turret pivot to hood pivot

        Pose3d turretPose = getTurretPose(subsytems.shooter.getTurretAngleDegrees());

        Pose3d hoodPosition =
                turretPose.transformBy(
                        new Transform3d(
                                dx,
                                dy,
                                dz,
                                new Rotation3d(
                                        0.0, hoodAngleDegrees * Conv.DEGREES_TO_RADIANS, 0.0)));
        return hoodPosition;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        FieldVisualizer.getInstance().testZeroedComponents();
        FieldVisualizer.getInstance()
                .updateTurret(
                        subsytems.shooter.getTurretAngleDegrees(),
                        subsytems.swerve.getState().Pose);
        Logger.recordOutput(
                "componentPoses",
                new Pose3d[] {
                    getTurretPose(subsytems.shooter.getTurretAngleDegrees()),
                    getHoodPose(subsytems.shooter.getHoodAngleDegrees())
                });
        Logger.recordOutput(
                "zeroedPoses",
                new Pose3d[] {
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0.0, 0))
                });

        if (kUseLimelight) {
            var driveState = subsytems.swerve.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            Pose2d currentPose =
                    subsytems.vision.getRobotPoseFromVision(headingDeg, omegaRps, 0, 0, 0, 0);

            if (currentPose != null) {
                subsytems.swerve.addVisionMeasurement(
                        currentPose,
                        subsytems.vision.getLastTimeStamp(),
                        VecBuilder.fill(
                                0.05, 0.05, 0.1)); // trusts vision rotation less. Needs tuning
                // increase the std devs to trust vision less
            }
        }
    }

    public void bindDriverController() {
        driverController.bind(subsytems);
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        // CommandScheduler.getInstance().getActiveButtonLoop().clear();
        CommandScheduler.getInstance().clearComposedCommands();
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
        if (fuelSim != null) {
            fuelSim.updateSim();

            // Logic to launch fuel when dispensing and shooter is ready
            double currentTime = RobotController.getFPGATime() / 1.0e6;
            if (subsytems.indexer.getExitRollerRPM() > 50.0
                    && subsytems.shooter.getCurrentState().rpm > 500.0
                    && (currentTime - lastShotTime) > 0.5) { // 0.5s cooldown

                var shooterState = subsytems.shooter.getCurrentState();

                // Launch parameters
                // Velocity is approx (RPM * radius / 2) because only one side is driven (per
                // AimSolver)
                double flywheelRadius = 0.0508; // 2 inches
                double launchVelocity =
                        (shooterState.rpm * 2.0 * Math.PI / 60.0 * flywheelRadius) / 2.0;

                fuelSim.launchFuel(
                        MetersPerSecond.of(launchVelocity),
                        Radians.of(Math.PI / 2 - shooterState.hoodAngleRads),
                        Radians.of(shooterState.turretAngleRads),
                        Meters.of(0.4) // height of shooter exit
                        );

                lastShotTime = currentTime;
                DogLog.log("Simulation/FuelLaunched", true);
            }
        }
    }

    private void configureFuelSim() {
        fuelSim = new FuelSim();
        fuelSim.spawnStartingFuel();
        fuelSim.start();
        SmartDashboard.putData(
                Commands.runOnce(
                                () -> {
                                    fuelSim.clearFuel();
                                    fuelSim.spawnStartingFuel();
                                })
                        .withName("Reset Fuel")
                        .ignoringDisable(true));

        configureFuelSimRobot();
    }

    private void configureFuelSimRobot() {
        // Chassis is approx 21x21 inches (0.53m). With bumpers, approx 28x28 (0.71m).
        double width = 0.71;
        double length = 0.71;
        double bumperHeight = 0.2;

        fuelSim.registerRobot(
                width,
                length,
                bumperHeight,
                () -> subsytems.swerve.getState().Pose,
                () -> subsytems.swerve.getState().Speeds);

        // Register a front intake zone (0.1m deep, 0.4m wide, centered in front of bumper)
        fuelSim.registerIntake(
                length / 2,
                length / 2 + 0.1,
                -0.2,
                0.2,
                () -> true,
                () -> DogLog.log("Simulation/FuelIntaked", true));
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
