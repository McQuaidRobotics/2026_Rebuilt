package igknighters;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.swerve.swerveconstants.GeminiConsts;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntegrationTest {
    private Robot robot;

    @BeforeEach
    public void setup() {
        try {
            HAL.initialize(500, 0);
        } catch (Exception e) {
        }

        // Reset the CommandScheduler
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();

        robot = new Robot(false);
        robot.robotInit();
    }

    @AfterEach
    public void tearDown() {
        robot.close();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    public void testRobotIntegration() {
        Subsystems subsystems = robot.subsystems;

        // --- 3. Test Manual Drive movement ---
        System.out.println("Starting Swerve Manual Test...");
        subsystems.swerve.resetPose(new Pose2d());
        double maxSpeed =
                GeminiConsts.kSpeedAt12Volts.in(MetersPerSecond);
        SwerveRequest.RobotCentric driveRequest =
                new SwerveRequest.RobotCentric()
                        .withDeadband(0)
                        .withRotationalDeadband(0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(maxSpeed * 0.5);

        for (int i = 0; i < 800; i++) {
            subsystems.swerve.setControl(driveRequest);

            if (i % 50 == 0) {
                System.out.println(
                        "Swerve Update -> X: "
                                + subsystems.swerve.getState().Pose.getX()
                                + ", Y: "
                                + subsystems.swerve.getState().Pose.getY()
                                + ", Rotation: "
                                + subsystems.swerve.getState().Pose.getRotation().getDegrees());
            }
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.autonomousPeriodic();
            try {
                Thread.sleep(1);
            } catch (Exception e) {
            }
        }

        double manualX = subsystems.swerve.getState().Pose.getX();
        System.out.println("Swerve Manual X: " + manualX);
        assertTrue(manualX > 0.1, "Robot should move during manual drive");

        // --- 4. Test Auto Routine ---
        System.out.println("Starting Swerve Auto Routine Test...");
        subsystems.swerve.resetPose(new Pose2d());
        var autoFactory = subsystems.swerve.createAutoFactory();
        var autoCommand = autoFactory.trajectoryCmd("ORBIT_RIGHT_1.traj");
        CommandScheduler.getInstance().schedule(autoCommand);

        for (int i = 0; i < 1000; i++) {
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.autonomousPeriodic();
            try {
                Thread.sleep(1);
            } catch (Exception e) {
            }
        }

        double autoX = subsystems.swerve.getState().Pose.getX();
        System.out.println("Auto Final X: " + autoX);
        assertTrue(Math.abs(autoX) > 0.1, "Robot should move during auto routine");
        autoCommand.cancel();
    }
}
