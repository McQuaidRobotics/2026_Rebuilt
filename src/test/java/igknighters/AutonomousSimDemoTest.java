package igknighters;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.util.AutonomousSimulation;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AutonomousSimDemoTest {
    private Robot robot;
    private AutonomousSimulation aiSim;

    @BeforeEach
    public void setup() {
        try {
            HAL.initialize(500, 0);
        } catch (Exception e) {
        }

        // Reset the CommandScheduler
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();

        // Initialize the robot
        robot = new Robot(false);
        aiSim = new AutonomousSimulation(robot);

        // Ensure we are in Teleop and Enabled
        aiSim.setEnabled(true);
        aiSim.setAutonomous(false);
    }

    @AfterEach
    public void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        aiSim.tearDown();
        robot.close();
    }

    @Test
    public void testDriveInCircle() {
        System.out.println("Starting Autonomous Simulation Circle Demo...");

        // Reset robot pose to origin
        robot.subsystems.swerve.resetPose(new Pose2d());

        // AI Agent Command: Drive in a circle
        // Constant forward velocity and constant rotational velocity
        aiSim.setForward(0.6);
        aiSim.setRotation(0.4);

        System.out.println("Applying AI Inputs: Forward=0.6, Rotation=0.4");

        // Run simulation for 200 ticks (approx 4 seconds at 20ms/tick)
        for (int i = 0; i < 200; i++) {
            double x = Math.sin(i / 200.0 * Math.PI * 2);
            double y = Math.cos(i / 200.0 * Math.PI * 2);
            aiSim.setForward(x);
            aiSim.setStrafe(y);
            // Update the simulation data
            DriverStationSim.notifyNewData();

            // Run robot cycles
            robot.robotPeriodic();
            robot.teleopPeriodic();

            // Advance the command scheduler
            CommandScheduler.getInstance().run();

            if (i % 50 == 0) {
                Pose2d pose = robot.subsystems.swerve.getState().Pose;
                System.out.println(
                        String.format(
                                "Tick %d -> X: %.2f, Y: %.2f, Heading: %.2f",
                                i, pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
            }

            try {
                Thread.sleep(1); // Small delay for test output readability
            } catch (InterruptedException e) {
            }
        }

        Pose2d finalPose = robot.subsystems.swerve.getState().Pose;
        System.out.println(
                String.format(
                        "Final Pose -> X: %.2f, Y: %.2f, Heading: %.2f",
                        finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees()));

        // Verification: The robot should have moved in both X and Y, and rotated
        assertTrue(Math.abs(finalPose.getX()) > 0.1, "Robot should have moved in X");
        assertTrue(Math.abs(finalPose.getY()) > 0.1, "Robot should have moved in Y");
        assertTrue(
                Math.abs(finalPose.getRotation().getDegrees()) > 10, "Robot should have rotated");

        System.out.println(
                "Demo Successful: Robot successfully navigated a circular path via AI simulation"
                        + " utility.");
    }
}
