package igknighters;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.constants.Conv;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.IntakeState;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
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

        robot = new Robot();
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
        Subsystems subsystems = robot.subsytems;

        // --- 1. Test Shooting movement ---
        System.out.println("Starting Shooter Test...");
        double targetTurretAngle = 15.0;
        double targetHoodAngle = 30.0;
        double targetRPM = 3000.0;

        for (int i = 0; i < 300; i++) {
            subsystems.shooter.targetState(targetRPM, targetTurretAngle, targetHoodAngle);
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.autonomousPeriodic();
        }

        double currentTurret =
                subsystems.shooter.getCurrentState().turretAngleRads * Conv.RADIANS_TO_DEGREES;
        double currentHood =
                subsystems.shooter.getCurrentState().hoodAngleRads * Conv.RADIANS_TO_DEGREES;
        double currentRPM = subsystems.shooter.getCurrentState().rpm;
        System.out.println(
                "Shooter Final -> Turret: "
                        + currentTurret
                        + ", Hood: "
                        + currentHood
                        + ", RPM: "
                        + currentRPM);

        assertTrue(Math.abs(currentTurret - targetTurretAngle) < 2.0, "Turret should move");
        assertTrue(Math.abs(currentHood - targetHoodAngle) < 10.0, "Hood should move");
        assertTrue(currentRPM > 1000, "Flywheel should spin");

        // --- 2. Test Intake movement ---

        // put intake in stowed position first
        subsystems.intake.setPivotDegrees(IntakeState.Stowed.pivotDegrees);

        System.out.println("Starting Intake Test...");

        System.out.println(
                "Intake Initial -> Pivot: "
                        + subsystems.intake.getPivotAngleDegrees()
                        + ", Roller RPM: "
                        + subsystems.intake.getRollerSpeedRPM());

        for (int i = 0; i < 200; i++) {
            subsystems.intake.goTo(IntakeState.Intake);
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.autonomousPeriodic();
        }
        System.out.println(
                "Intake Final -> Pivot: "
                        + subsystems.intake.getPivotAngleDegrees()
                        + ", Roller RPM: "
                        + subsystems.intake.getRollerSpeedRPM());

        boolean intakeMoved =
                !subsystems.intake.isAt(
                        IntakeState.Stowed.pivotDegrees,
                        IntakeState.Stowed.rollerSpeedRPM,
                        10.0,
                        100.0);
        System.out.println("Intake Moved from Stowed: " + intakeMoved);
        assertTrue(intakeMoved, "Intake should have moved away from stowed position");

        // --- 3. Test Manual Drive movement ---
        System.out.println("Starting Swerve Manual Test...");
        subsystems.swerve.resetPose(new Pose2d());
        double maxSpeed = knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond);
        SwerveRequest.RobotCentric driveRequest =
                new SwerveRequest.RobotCentric()
                        .withDeadband(0)
                        .withRotationalDeadband(0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(maxSpeed * 0.5);

        for (int i = 0; i < 200; i++) {
            subsystems.swerve.setControl(driveRequest);
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
        var autoCommand = autoFactory.trajectoryCmd("ShootThenIntake");
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
