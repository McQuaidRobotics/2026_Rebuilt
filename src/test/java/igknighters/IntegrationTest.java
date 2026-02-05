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
import igknighters.commands.IntakeCommands;
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

        // --- 1. Test Shooting movement (Turret + Hood + Flywheel) ---
        double targetTurretAngle = 15.0;
        double targetHoodAngle = 30.0;
        double targetRPM = 3000.0;

        System.out.println("Starting Shooter Test...");
        var shootCommand =
                subsystems.shooter.run(
                        () ->
                                subsystems.shooter.targetState(
                                        targetRPM, targetTurretAngle, targetHoodAngle));
        shootCommand.schedule();

        // Run loops
        for (int i = 0; i < 100; i++) {
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

        assertTrue(
                Math.abs(currentTurret - targetTurretAngle) < 2.0, "Turret should be near target");
        assertTrue(
                Math.abs(currentHood - targetHoodAngle) < 10.0,
                "Hood should have moved towards target");
        assertTrue(currentRPM > 1000, "Flywheel should be spinning");

        shootCommand.cancel();

        // --- 2. Test Intake movement ---
        System.out.println("Starting Intake Test...");
        subsystems.intake.goTo(IntakeState.Stowed);
        for (int i = 0; i < 5; i++) {
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
        }

        var intakeCommand = IntakeCommands.goToIntake(subsystems.intake);
        intakeCommand.schedule();

        for (int i = 0; i < 100; i++) {
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.autonomousPeriodic();
        }

        assertTrue(
                subsystems.intake.isAt(
                        IntakeState.Intake.pivotDegrees,
                        IntakeState.Intake.rollerSpeedRPM,
                        10.0,
                        500.0),
                "Intake should be near target position and speed");

        intakeCommand.cancel();

        // --- 3. Test Manual Drive movement ---

        System.out.println("Starting Swerve Manual Test (Teleop)...");

        DriverStationSim.setAutonomous(false);

        DriverStationSim.notifyNewData();

        // Run some loops to let Swerve initialize with mode change

        for (int i = 0; i < 10; i++) {

            DriverStationSim.notifyNewData();

            robot.robotPeriodic();

            robot.teleopPeriodic();
        }

        subsystems.swerve.resetPose(new Pose2d());

        double maxSpeed = knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond);

        SwerveRequest.FieldCentric driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(0)
                        .withRotationalDeadband(0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(maxSpeed * 0.5);

        var driveCommand = subsystems.swerve.run(() -> subsystems.swerve.setControl(driveRequest));

        driveCommand.schedule();

        for (int i = 0; i < 200; i++) {

            DriverStationSim.notifyNewData();

            robot.robotPeriodic();

            robot.teleopPeriodic();

            try {
                Thread.sleep(1);
            } catch (Exception e) {
            }
        }

        double manualX = subsystems.swerve.getState().Pose.getX();

        System.out.println("Swerve Manual X: " + manualX);

        assertTrue(manualX > 0.1, "Robot should have moved forward during manual drive");

        driveCommand.cancel();

        // --- 4. Test Auto Routine ---

        System.out.println("Starting Swerve Auto Routine Test (Autonomous)...");

        DriverStationSim.setAutonomous(true);

        DriverStationSim.notifyNewData();

        for (int i = 0; i < 10; i++) {

            DriverStationSim.notifyNewData();

            robot.robotPeriodic();

            robot.autonomousPeriodic();
        }

        subsystems.swerve.resetPose(new Pose2d());

        var autoFactory = subsystems.swerve.createAutoFactory();

        var autoCommand = autoFactory.trajectoryCmd("ShootThenIntake");

        autoCommand.schedule();

        for (int i = 0; i < 1000; i++) {

            DriverStationSim.notifyNewData();

            robot.robotPeriodic();

            robot.autonomousPeriodic();

            if (i % 100 == 0)
                System.out.println(
                        "Auto Loop " + i + ", X: " + subsystems.swerve.getState().Pose.getX());

            try {
                Thread.sleep(1);
            } catch (Exception e) {
            }
        }

        double autoX = subsystems.swerve.getState().Pose.getX();
        System.out.println("Auto Final X: " + autoX);
        assertTrue(Math.abs(autoX) > 0.1, "Robot should have moved during auto routine");
        autoCommand.cancel();
    }
}
