package igknighters.commands.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.Robot;
import igknighters.constants.GeminiRobotConsts;
import igknighters.subsystems.shooter.Shooter;
import org.junit.jupiter.api.Test;

public class HomeHoodTest {

    @Test
    public void testHomeHoodResetsEncoderAndStopsMotor() {
        // Ensure we have constants available for the test
        Robot.consts = new GeminiRobotConsts();

        Shooter shooter = new Shooter();

        // Confirm simulated hood starts at 0deg which is <= MIN_ANGLE and should trigger short
        // circuit
        Command cmd = ShooterCommands.homeHood(shooter);

        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.schedule(cmd);

        // Run scheduler for a few cycles to allow runOnce to execute
        for (int i = 0; i < 5; i++) {
            scheduler.run();
        }

        assertEquals(
                Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES(),
                shooter.getHoodAngleDegrees(),
                1e-6);

        scheduler.cancelAll();
    }
}
