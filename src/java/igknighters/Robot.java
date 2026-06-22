// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import igknighters.commands.IndexerCommands;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.Conv;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.constants.GeminiRobotConsts;
import igknighters.constants.RobotConsts;
import igknighters.constants.RobotIdentity;
import igknighters.constants.SecondBotRobotConsts;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.YamShooter.Shooter;
import igknighters.subsystems.YamsIntake.YamIntake;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.FuelSim;
import igknighters.util.RobotPosePredError;
import igknighters.util.RobotPosePredictor;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.TurretPosePredError;
import igknighters.util.TurretPosePredictor;
import igknighters.util.log.Log;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;

    public static RobotConsts consts;
    private AutoFactory autoFactory;
    public final AutoChooser autoChooser = new AutoChooser();
    public final AutoChooser testChooser = new AutoChooser();
    double i = 0;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final SubsystemTriggers subsystemTriggers = new SubsystemTriggers();
    public static RobotPosePredictor pose_pred;
    public static TurretPosePredictor turret_pred = new TurretPosePredictor();
    public static RobotPosePredError pose_pred_error = new RobotPosePredError();
    public static TurretPosePredError turret_pred_error = new TurretPosePredError();

    private final DriverController driverController = new DriverController(0);

    public final Subsystems subsystems;

    private FuelSim fuelSim;
    private double lastShotTime = 0.0;

    private final boolean kUseLimelight = true;

    private Telemetry logger;
    TunableDouble detune = TunableValues.getDouble("Tunables/Detune", 1.0);
    TunableDouble targetingP = TunableValues.getDouble("Tunables/TargetingP", 0.07);
    TunableDouble targetingI = TunableValues.getDouble("Tunables/TargetingI", 0.00);
    TunableDouble targetingD = TunableValues.getDouble("Tunables/TargetingD", 0.00);

    public void setUpCommandLogging() {
        if (!Robot.consts.disableAllLogs()) {
            scheduler.onCommandInitialize(
                    command ->
                            Log.log(
                                    "Commands/Tracking/" + command.getName() + "/ Command Running",
                                    "TRUE"));

            scheduler.onCommandInitialize(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "FALSE"));

            scheduler.onCommandInterrupt(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "TRUE"));
            scheduler.onCommandFinish(
                    command ->
                            Log.log(
                                    "Commands/Tracking/" + command.getName() + "/ Command Running",
                                    "FALSE"));
            scheduler.onCommandFinish(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "FALSE"));
        }
    }

    public void publishCommandsAndSubystems(Subsystems subsystems) {
        SmartDashboard.putData(CommandScheduler.getInstance());
        for (var subsystem : subsystems.lockedResources) {
            SmartDashboard.putData("SubsystemCommands/" + subsystem.getName(), subsystem);
        }
    }

    public void setUpRobotConsts() {

        // THE IDS WILL BE WRONG SINCE SN IS WRONG WILL DEFAULT TO SECOND BOT
        if (RobotIdentity.isGemini()) {
            consts = new GeminiRobotConsts();
        } else if (RobotIdentity.isSecondBot()) {
            consts = new SecondBotRobotConsts();
        } else if (Robot.isReal()) {
            throw new IllegalStateException(
                    "Unknown robot identity ENSURE SERIAL NUMBERS MATCH"); // only problem irl
        } else {
            consts = new GeminiRobotConsts(); // in sim with unknown sn we should pick something
        }
    }

    public void setUpAutos(Subsystems subsystems) {
        autoFactory = subsystems.swerve.createAutoFactory();
        final var routines = new AutoRoutines(subsystems, autoFactory, consts);
        autoChooser.addRoutine("Right Orbit", routines::orbitRight);
        autoChooser.addRoutine("Left Orbit", routines::ORBIT_LEFT);
        autoChooser.addRoutine("Center Depot", routines::centerPreload);
        autoChooser.addRoutine("LEFT BUMP PASS TO SELF", routines::BUMP_PASS_TO_SELF_LEFT);
        autoChooser.addRoutine(
                "Pass to Self Right with Depot and Human Player",
                routines::PASS_TO_SELF_RIGHT_WITH_DEPOT_AND_HUMAN_PLAYER);
        autoChooser.addRoutine("OP RIGHT", routines::OP_RIGHT);
        autoChooser.addRoutine("OP_LEFT", routines::OP_LEFT);

        testChooser.addRoutine("Test Auto", routines::TEST);

        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
        SmartDashboard.putData("TEST CHOOSER", testChooser);
    }

    public void setUpSwerve(Subsystems subsystems) {
        subsystems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsystems.swerve, driverController, 1.0));

        logger = new Telemetry(subsystems.swerve.getMaxSpeedMetersPerSecond(), subsystems);
        subsystems.swerve.registerTelemetry(logger::telemeterize);
    }

    public void setUpTest(Subsystems subsystems) {
        SmartDashboard.putData(
                "Commands/Spindexer/Spindexer - STOP",
                IndexerCommands.justStop(subsystems.indexer));
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
        setUpRobotConsts();
        setUpAdvantageScope();
        setUpCommandLogging();
        subsystems =
                new Subsystems(
                        new Swerve(false),
                        new LimeLightVision(),
                        new Led(90, 2),
                        new Shooter(),
                        new Indexer(),
                        new YamIntake(),
                        new Luma(true, "object-detection"));
        setUpSwerve(subsystems);
        publishCommandsAndSubystems(subsystems);
        setUpAutos(subsystems);
        setUpTest(subsystems);
        bindDriverController();

        pose_pred = new RobotPosePredictor(subsystems.swerve);

        subsystemTriggers.SetupTriggers(subsystems, driverController, poseSupplier());

        if (isSimulation()) {
            configureFuelSim();
        }
    }

    public Robot(boolean isSwerveDisabled) {
        setUpRobotConsts();
        setUpAdvantageScope();
        setUpCommandLogging();
        subsystems =
                new Subsystems(
                        new Swerve(isSwerveDisabled),
                        new LimeLightVision(),
                        new Led(90, 2),
                        new Shooter(),
                        new Indexer(),
                        new YamIntake(),
                        new Luma(true, "object-detection"));
        setUpSwerve(subsystems);
        pose_pred = new RobotPosePredictor(subsystems.swerve);
        publishCommandsAndSubystems(subsystems);
        setUpAutos(subsystems);
        setUpTest(subsystems);
        bindDriverController();

        subsystemTriggers.SetupTriggers(subsystems, driverController, poseSupplier());
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

    public Supplier<Pose2d> poseSupplier() {
        return () -> subsystems.swerve.getState().Pose;
    }

    public Pose3d getHoodPose(double hoodAngleDegrees) {
        double dx = 0.09; // X offset from turret center to hood
        double dy = 0.0; // Y offset from turret center to hood
        double dz = 0.12; // z offset from turret pivot to hood pivot

        Pose3d turretPose = getTurretPose(subsystems.shooter.turret.getAngle().in(Degrees));

        Pose3d hoodPosition =
                turretPose.transformBy(
                        new Transform3d(
                                dx,
                                dy,
                                dz,
                                new Rotation3d(
                                        0.0, hoodAngleDegrees * Conv.DEGREES_TO_RADIANS, 0)));
        return hoodPosition;
    }

    boolean underTrench() {
        Pose2d turretPredPose = turret_pred.getPredictedPose().get().toPose2d();
        Pose2d turretAccPose = subsystems.swerve.getState().Pose;

        double dx1Pred = Math.abs(turretPredPose.getX() - FieldConstants.BUMP.BUMP_1_X_METERS);
        double dx2Pred = Math.abs(turretPredPose.getX() - FieldConstants.BUMP.BUMP_2_X_METERS);

        double dx1Acc = Math.abs(turretAccPose.getX() - FieldConstants.BUMP.BUMP_1_X_METERS);
        double dx2Acc = Math.abs(turretAccPose.getX() - FieldConstants.BUMP.BUMP_2_X_METERS);

        boolean under1Pred = dx1Pred <= .5;
        boolean under2Pred = dx2Pred <= .5;

        boolean under1Acc = dx1Acc <= .5;
        boolean under2Acc = dx2Acc <= .5;

        boolean isUnder = under1Pred || under2Pred || under1Acc || under2Acc;
        return isUnder;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // // THE COORDINATES LOOK WEIRD WHEN THERE ARE MULTIPLE FUEL, needs tuning
        // Log.log(
        //         "Subsystems/Vision/ObjectDetection/Closest Game Piece",
        //         subsystems.luma.getClosestGamePiece());

        pose_pred.setVelocitiesAndPose();
        subsystems.shooter.periodic();

        if (underTrench()) {
            DrivingSharedState.getInstance().setUnderTrench(true);
        } else {
            DrivingSharedState.getInstance().setUnderTrench(false);
        }
        turret_pred.logTurretPose(
                turret_pred.getTurretPoseFieldRelativeOffset(subsystems.swerve.getState().Pose));
        pose_pred_error.logPose(subsystems.swerve.getState().Pose);
        turret_pred_error.logPose(
                turret_pred.getTurretPoseFieldRelativeOffset(subsystems.swerve.getState().Pose));
        if (Robot.isReal() && !consts.disableAllLogs()) {
            FieldVisualizer.getInstance()
                    .updateTurret(
                            subsystems.shooter.turret.getAngle().in(Degrees),
                            subsystems.swerve.getState().Pose);
            Logger.recordOutput(
                    "componentPoses",
                    new Pose3d[] {
                        getTurretPose(subsystems.shooter.turret.getAngle().in(Degrees)),
                        getHoodPose(subsystems.shooter.hood.getAngle().in(Degrees))
                    });
        } else {
            FieldVisualizer.getInstance()
                    .updateTurret(
                            subsystems.shooter.turret.getAngle().in(Degrees),
                            subsystems.swerve.getState().Pose);
            Logger.recordOutput(
                    "componentPoses",
                    new Pose3d[] {
                        getTurretPose(subsystems.shooter.turret.getAngle().in(Degrees)),
                        getHoodPose(subsystems.shooter.hood.getAngle().in(Degrees))
                    });
        }

        Logger.recordOutput(
                "zeroedPoses",
                new Pose3d[] {
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0.0, 0))
                });

        if (kUseLimelight) {
            var driveState = subsystems.swerve.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            Pose2d currentPose =
                    subsystems.vision.getRobotPoseFromVision(headingDeg, omegaRps, 0, 0, 0, 0);

            if (currentPose != null) {
                subsystems.swerve.addVisionMeasurement(
                        currentPose,
                        subsystems.vision
                                .getLastTimeStamp()); // trusts vision rotation less. Needs tuning
                // increase the std devs to trust vision less
                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log("ROBOT/Subsystems/Vision/Null Pose", false);
                }
            } else {
                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log("ROBOT/Subsystems/Vision/Null Pose", true);
                }
            }
        }
    }

    public void bindDriverController() {
        driverController.bind(subsystems);
    }

    @Override
    public void disabledInit() {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        subsystems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsystems.swerve, driverController, detune.value()));
        DrivingSharedState.getInstance().setDetune(detune.value());
        DrivingSharedState.getInstance().setKP(targetingP.value());
        DrivingSharedState.getInstance().setKI(targetingI.value());
        DrivingSharedState.getInstance().setKD(targetingD.value());
        subsystems.vision.disableCameras();

        bindDriverController();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        subsystems.vision.enableCameras(0);
    }

    @Override
    public void autonomousInit() {

        subsystems.vision.enableCameras(0);
        Command autoCommand = autoChooser.selectedCommand();
        if (fuelSim != null) {
            fuelSim.start();
        }
        try {
            scheduler.schedule(ShooterCommands.homeHood(subsystems.shooter));
        } catch (Exception e) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/autonomousInit/HomeHoodScheduleFailed", e.toString());
            }
        }
        m_autonomousCommand = autoCommand;
        if (autoCommand != null) {
            scheduler.schedule(autoCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        subsystems.swerve.clearActiveTrajectory();
        scheduler.cancelAll();
    }

    @Override
    public void teleopInit() {
        subsystems.vision.enableCameras(0);
        subsystems.swerve.clearActiveTrajectory();
        if (fuelSim != null) {
            fuelSim.start();
        }
        scheduler.cancelAll();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // Schedule a homing command for the hood when teleop starts so the hood is
        // zeroed before driver control. This will be a no-op if the hood sensor is
        // already triggered because homeHood handles the short-circuit case.
        try {
            scheduler.schedule(ShooterCommands.homeHood(subsystems.shooter));
        } catch (Exception e) {
            // Log but don't crash the robot if scheduling fails for any reason.
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/teleopInit/HomeHoodScheduleFailed", e.toString());
            }
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Command autoCommand = testChooser.selectedCommand();
        if (fuelSim != null) {
            fuelSim.start();
        }
        scheduler.schedule(autoCommand);
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    public static boolean isRobotTest() {
        return RobotModeTriggers.test().getAsBoolean();
    }

    @Override
    public void simulationPeriodic() {
        if (fuelSim != null) {
            fuelSim.updateSim();

            // Logic to launch fuel when dispensing and shooter is ready
            double currentTime = RobotController.getFPGATime() / 1.0e6;
            boolean exitRollersActive = subsystems.exitRollers.getVelocity().in(RPM) > 50.0;
            boolean shooterReady =
                    subsystems.shooter.getCurrentState().flywheelSpeed.in(RPM) > 500.0;
            boolean spindexerActive = subsystems.spindexer.getVelocity().in(RPM) > 50.0;
            boolean timeDeltaValid = (currentTime - lastShotTime) > 0.1;
            if (exitRollersActive
                    && shooterReady
                    && spindexerActive
                    && timeDeltaValid) { // 0.1s cooldown
                var shooterState = subsystems.shooter.getCurrentState();

                // Launch parameters
                // Velocity is approx (RPM * radius / 2) because only one side is driven (per
                // AimSolver)
                double flywheelRadius = Robot.consts.shooter().kFlywheels().WHEEL_RADIUS_METERS();
                double launchVelocity =
                        (shooterState.flywheelSpeed.in(RadiansPerSecond) * flywheelRadius) / 2.2;

                fuelSim.launchFuel(
                        MetersPerSecond.of(launchVelocity),
                        Radians.of(Math.PI / 2 - shooterState.hoodAngle.in(Radian)),
                        shooterState.turretAngle,
                        Meters.of(
                                Robot.consts
                                        .shooter()
                                        .kFlywheels()
                                        .ShooterHeightMeters()) // height of shooter exit
                        );

                lastShotTime = currentTime;
                if (!Robot.consts.shooter().kFlywheels().disableFlywheelsLogs()) {
                    Log.log("ROBOT/Simulation/FuelLaunched", true);
                }
            }
        }
    }

    private void configureFuelSim() {
        fuelSim = new FuelSim();
        // fuelSim.spawnStartingFuel();
        fuelSim.start();
        SmartDashboard.putData(
                Commands.runOnce(
                                () -> {
                                    fuelSim.clearFuel();
                                })
                        .withName("Clear Fuel")
                        .ignoringDisable(true));
        fuelSim.enableAirResistance();

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
                () -> subsystems.swerve.getState().Pose,
                subsystems.swerve::getFieldRelativeSpeeds);

        // Register a front intake zone (0.1m deep, 0.4m wide, centered in front of bumper)
        fuelSim.registerIntake(
                length / 2,
                length / 2 + 0.1,
                -0.2,
                0.2,
                () -> true,
                () -> Log.log("ROBOT/Simulation/FuelIntaked", true));
    }

    public static boolean isBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent()) {
            return ally.get() == Alliance.Blue;
        } else {
            // Default to blue if alliance is unknown (e.g., in simulation without alliance set)
            // Log this so we know why things might be going to the blue side.
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/System/AllianceUnknown", true);
            }
            return true;
        }
    }

    public static boolean isReplay() {
        return false; // change to true to replay matches
    }
}
