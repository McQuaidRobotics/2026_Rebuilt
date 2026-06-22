package igknighters;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.commands.Shooter.ShooterCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.YamsIntake.YamIntakeState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IntegrationTest {
    private static Robot robot;

    @BeforeAll
    public static void setupAll() {
        robot = TestRobot.get();
    }

    @Test
    public void testEverything() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        // --- 1. Home Hood ---
        System.out.println("Starting Hood Homing Test...");
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().removeDefaultCommand(robot.subsystems.shooter.hood);
        CommandScheduler.getInstance().removeDefaultCommand(robot.subsystems.shooter.flywheels);
        CommandScheduler.getInstance().removeDefaultCommand(robot.subsystems.shooter.turret);

        Command homeCmd = ShooterCommands.homeHood(robot.subsystems.shooter);
        CommandScheduler.getInstance().schedule(homeCmd);

        for (int i = 0; i < 500; i++) {
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.simulationPeriodic();
            if (homeCmd.isFinished()) break;
        }
        System.out.println(
                "Hood Final Angle: "
                        + robot.subsystems.shooter.getCurrentState().hoodAngle.in(Degrees));

        // --- 2. Robot Integration (Shooter/Intake/Swerve) ---
        System.out.println("Starting Robot Integration Test...");
        Subsystems subsystems = robot.subsystems;
        subsystems.swerve.resetPose(new Pose2d());

        double targetTurretAngle = 15.0;
        double targetHoodAngle = 30.0;
        double targetRPM = 3000.0;

        for (int i = 0; i < 500; i++) {
            subsystems.shooter.targetState(
                    RPM.of(targetRPM), Degrees.of(targetTurretAngle), Degrees.of(targetHoodAngle));
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.simulationPeriodic();
        }

        double currentTurret = subsystems.shooter.getCurrentState().turretAngle.in(Degrees);
        double currentHood = subsystems.shooter.getCurrentState().hoodAngle.in(Degrees);
        double currentRPM = subsystems.shooter.getCurrentState().flywheelSpeed.in(RPM);
        System.out.println(
                "Shooter Final -> Turret: "
                        + currentTurret
                        + ", Hood: "
                        + currentHood
                        + ", RPM: "
                        + currentRPM);

        assertTrue(currentRPM > 1000, "Flywheel should spin");

        // --- 3. Intake ---
        subsystems.intake.targetState(YamIntakeState.STOWED);
        for (int i = 0; i < 200; i++) {
            subsystems.intake.targetState(YamIntakeState.DEPLOYED);
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.simulationPeriodic();
        }
        assertTrue(
                !subsystems.intake.isAt(YamIntakeState.DEPLOYED, Degrees.of(10.0), RPM.of(100.0)),
                "Intake should move");

        // --- 4. Swerve Manual ---
        subsystems.swerve.resetPose(new Pose2d());
        double maxSpeed =
                Robot.consts.swerve().getCommonSwerveConsts().getMaxSpeedMetersPerSecond();
        SwerveRequest.RobotCentric driveRequest =
                new SwerveRequest.RobotCentric().withVelocityX(maxSpeed * 0.5);
        for (int i = 0; i < 200; i++) {
            subsystems.swerve.setControl(driveRequest);
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.simulationPeriodic();
        }
        assertTrue(subsystems.swerve.getState().Pose.getX() > 0.1, "Robot should move");
    }
}
