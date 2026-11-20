package igknighters;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.commands.Shooter.Sentry;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/** Headless run of the sim demo: sentry should hit kids in front and never the ones behind. */
public class SentryDemoTest {
    private Robot robot;

    @AfterEach
    public void tearDown() {
        robot.close();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    public void sentryHitsFrontKidsOnly() {
        HAL.initialize(500, 0);
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();

        robot = new Robot(false);
        assertNotNull(robot.sentrySim, "DEMO_MODE sim should build SentrySim");
        var s = robot.subsystems;
        CommandScheduler.getInstance()
                .schedule(new Sentry(s.shooter, s.indexer, () -> 0.0, () -> false));

        int fireTicks = 0;
        for (int i = 0; i < 50 * 20; i++) { // 20 s of sim time
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            robot.simulationPeriodic();
            if (s.indexer.getExitRollerRPM() > 50.0) { // feeding = a ball is leaving
                fireTicks++;
                double turret = s.shooter.getTurretAngleDegrees();
                assertTrue(Sentry.inArc(turret), "fired with turret outside front arc: " + turret);
            }
            try {
                Thread.sleep(20); // real time: stepTiming deadlocks with the swerve sim thread
            } catch (InterruptedException e) {
            }
            if (i % 250 == 0) {
                System.out.printf(
                        "t=%.0fs front hits=%d behind hits=%d turret=%.1f%n",
                        i * 0.02,
                        robot.sentrySim.hits(false),
                        robot.sentrySim.hits(true),
                        s.shooter.getTurretAngleDegrees());
            }
        }
        System.out.printf(
                "final: fire ticks=%d front hits=%d behind hits=%d%n",
                fireTicks, robot.sentrySim.hits(false), robot.sentrySim.hits(true));
        // hit count depends on Sentry/RPM + HoodDeg tuning (fixed, not distance-solved yet),
        // so only assert what sentry owns: it fires, only in the arc, never at the kids behind
        assertTrue(fireTicks > 0, "sentry should lock on and fire");
        assertEquals(0, robot.sentrySim.hits(true), "must never hit kids behind");
    }
}
