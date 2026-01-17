package igknighters.commands.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;

public class AutoRoutines extends AutoCommands {

    public AutoRoutines(Subsystems subsystems, AutoFactory factory) {
        super(subsystems, factory);

        if (Robot.isSimulation()) {
            new Trigger(DriverStation::isAutonomousEnabled)
                    .onTrue(
                            Commands.waitSeconds(15.3)
                                    .andThen(() -> DriverStationSim.setEnabled(false))
                                    .withName("Simulated Auto Ender"));
        }
    }

    public Supplier<Command> trajTest(String trajName) {
        return () ->
                Commands.sequence(
                        autoFactory.resetOdometry(trajName), autoFactory.trajectoryCmd(trajName));
    }

    @FunctionalInterface
    public interface DualSideAuto {
        Command generate();
    }

    public static void addCmd(AutoChooser chooser, String name, DualSideAuto auto) {
        chooser.addCmd(name, () -> auto.generate());
    }

    // public Command driveAround() {
    //     return newAuto("shoot_then_pass")
    //             .addDrivingTrajectory(
    //                     Waypoints.STARTING_RIGHT, Waypoints.BUMP_LAND_RIGHT, Waypoints.BALLS_RIGHT)
    //             .build();
    // }

    public Supplier<Command> shootThenMove() {
        return () ->
                newRebuiltAuto("SHOOT-THEN-MOVE")
                        .shootThenMove(Waypoints.STARTING_RIGHT, Waypoints.BUMP_LAND_RIGHT, 5.0)
                        .addDrivingTrajectory(Waypoints.BUMP_LAND_RIGHT, Waypoints.BALLS_RIGHT)
                        .shootAndMove(Waypoints.BALLS_RIGHT, Waypoints.BALLS_MIDDLE)
                        .build();
    }

    public Command rightToLeft() {
        return newRebuiltAuto("right to left")
                .shootAndMove(Waypoints.STARTING_RIGHT, Waypoints.BUMP_LAND_RIGHT)
                .shootAndMove(Waypoints.BALLS_RIGHT, Waypoints.BALLS_MIDDLE)
                .build();
    }
}
