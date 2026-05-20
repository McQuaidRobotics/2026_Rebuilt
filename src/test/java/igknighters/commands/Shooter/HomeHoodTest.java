package igknighters.commands.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.Robot;
import igknighters.constants.GeminiRobotConsts;
import igknighters.subsystems.shooter.Shooter;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class HomeHoodTest {
    private Robot robot;
    private Shooter shooter;

    @BeforeEach
    public void setup() {
        // 1. Kill the scheduler
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();

        // 2. Clear out the hardware simulators
        edu.wpi.first.hal.HAL.initialize(500, 0);
        edu.wpi.first.wpilibj.simulation.SimDeviceSim.resetData();

        Robot.consts = new GeminiRobotConsts();
        robot = new Robot(false);
        shooter = robot.subsystems.shooter;

        // Ensure the robot is enabled so commands can run
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
    }

    @AfterEach
    public void tearDown() {
        if (robot != null) {
            robot.close();
        }
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    public void testHomeHoodResetsEncoderAndStopsMotor() {
        Command cmd = ShooterCommands.homeHood(shooter);
        CommandScheduler.getInstance().schedule(cmd);

        // Run scheduler for a few cycles
        for (int i = 0; i < 5; i++) {
            CommandScheduler.getInstance().run();
        }

        assertEquals(
                Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES(),
                shooter.getHoodAngleDegrees(),
                1e-6);
    }
}
