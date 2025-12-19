// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.DrivingSharedState;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightVisionConstants;
import igknighters.subsystems.LimeLightVision.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.LimeLightVisionSim;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final AutoFactory autoFactory;
    public final AutoChooser autoChooser = new AutoChooser();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final SubsystemTriggers subsystemTriggers = new SubsystemTriggers();

    private final DriverController driverController = new DriverController(0);

    public final Subsystems subsytems;

    private final boolean kUseLimelight = true;

    private final SwerveConsts swerveConstGetter = new SwerveConsts();

    private final CommonSwerveConsts swerveConsts = swerveConstGetter.getSwerveConsts();

    private final Telemetry logger;
    TunableDouble detune = TunableValues.getDouble("Tunables/Detune", 0.6);
    TunableDouble targetingP = TunableValues.getDouble("Tunables/TargetingP", 0.07);
    TunableDouble targetingI = TunableValues.getDouble("Tunables/TargetingI", 0.00);
    TunableDouble targetingD = TunableValues.getDouble("Tunables/TargetingD", 0.00);

    public Robot() {
        if (Robot.isReal()) {
            subsytems =
                    new Subsystems(
                            swerveConsts.createDrivetrain(),
                            new LimeLightVisionReal(LimelightVisionConstants.backLeft),
                            new Led(40, 1));
        } else {
            subsytems =
                    new Subsystems(
                            swerveConsts.createDrivetrain(),
                            new LimeLightVisionSim(),
                            new Led(40, 1));
        }
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, .8));

        logger = new Telemetry(swerveConsts.getMaxSpeedMetersPerSecond(), subsytems);
        subsytems.swerve.registerTelemetry(logger::telemeterize);
        driverController.bind(subsytems);
        autoFactory = subsytems.swerve.createAutoFactory();
        final var routines = new AutoRoutines(subsytems, autoFactory);
        AutoRoutines.addCmd(autoChooser, "ZOOOOOOOOMMMMMMM", routines::driveAround);
        autoChooser.addCmd("TRAJECTORY TEST", routines.trajTest("Straight"));
        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
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

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, detune.value()));
        DrivingSharedState.getInstance().setDetune(detune.value());
        DrivingSharedState.getInstance().setKP(targetingP.value());
        DrivingSharedState.getInstance().setKI(targetingI.value());
        DrivingSharedState.getInstance().setKD(targetingD.value());

        driverController.bind(subsytems);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Command autoCmd = autoChooser.selectedCommand();
        String msg = "---- Starting auto command: " + autoCmd.getName() + " ----";
        DogLog.log("AutoEvent", msg);
        scheduler.schedule(autoCmd);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        scheduler.cancelAll();
    }

    @Override
    public void teleopInit() {
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
        for (var subsystem : subsytems.locklessResources) {
            subsystem.simulationPeriodic();
        }
    }
}
