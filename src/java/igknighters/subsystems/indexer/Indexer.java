package igknighters.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.indexer.launcherRollers.ExitRollers;
import igknighters.subsystems.indexer.launcherRollers.ExitRollersReal;
import igknighters.subsystems.indexer.launcherRollers.ExitRollersSim;
import igknighters.subsystems.indexer.spindexer.Spindexer;
import igknighters.subsystems.indexer.spindexer.SpindexerReal;
import igknighters.subsystems.indexer.spindexer.SpindexerSim;

public class Indexer extends SubsystemBase {
    private Spindexer spindexer;
    private ExitRollers exitRollers;

    public Indexer() {
        if (Robot.isReal()) {
            spindexer = new SpindexerReal();
            exitRollers = new ExitRollersReal();

        } else {
            spindexer = new SpindexerSim();
            exitRollers = new ExitRollersSim();
        }
    }

    public void setRPM(double RPM) {
        spindexer.goToRPM(RPM);
    }

    public double getRPM() {
        return spindexer.getRPM();
    }

    public void goToState(IndexerState state) {
        spindexer.goToRPM(state.spindexerRPM);
        exitRollers.setSpeedRPM(state.exitRollerRPM);
    }

    public void stop() {
        spindexer.stop();
    }

    @Override
    public void periodic() {
        spindexer.periodic();
        exitRollers.periodic();
    }
}
