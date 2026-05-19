package igknighters.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.indexer.launcherRollers.*;
import igknighters.subsystems.indexer.spindexer.*;

public class Indexer {
    public ExitRollersBase exitRollers;
    public SpindexerBase spindexer;

    public Indexer() {
        spindexer = new SpindexerFunctioning();
        exitRollers = new ExitRollersFunctioning();
    }

    public Command goToState(IndexerState state) {
        return Commands.parallel(
                spindexer
                        .setVelocity(RPM.of(state.getSpindexerRPM()))
                        .withName("SET SPEED SPINDEXER AT, " + state.getSpindexerRPM() + " RPM"),
                exitRollers
                        .setVelocity(RPM.of(state.getExitRollerRPM()))
                        .withName(
                                "SET SPEED EXIT ROLLERS AT, " + state.getExitRollerRPM() + " RPM"));
    }

    public void goToStateNotCommand(IndexerState state) {
        spindexer.setVelocitySetpoint(RPM.of(state.getSpindexerRPM()));
        exitRollers.setVelocitySetpoint(RPM.of(state.getExitRollerRPM()));
    }

    public Command idleSpindexer() {
        return Commands.repeatingSequence(
                        spindexer.setVelocity(RPM.of(1000)).withTimeout(.2),
                        spindexer.setVelocity(RPM.of(0.0)).withTimeout(.2),
                        spindexer.setVelocity(RPM.of(-1000)).withTimeout(.2),
                        spindexer.setVelocity(RPM.of(0.0)).withTimeout(.2))
                .withName("IDLE SPINDEXER - DEFAULT COMMAND");
    }
}
