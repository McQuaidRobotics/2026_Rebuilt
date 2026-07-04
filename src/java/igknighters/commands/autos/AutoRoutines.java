package igknighters.commands.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.constants.RobotConsts;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;

public class AutoRoutines extends AutoCommands {

    RobotConsts consts;

    public AutoRoutines(Subsystems subsystems, AutoFactory factory, RobotConsts consts) {
        super(subsystems, factory);
        this.consts = consts;

        if (Robot.isSimulation()) {
            new Trigger(DriverStation::isAutonomousEnabled)
                    .onTrue(
                            Commands.waitSeconds(20.0)
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
}
