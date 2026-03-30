package igknighters.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.indexer.launcherRollers.*;
import igknighters.subsystems.indexer.launcherRollers.ExitRollers;
import igknighters.subsystems.indexer.spindexer.*;
import igknighters.subsystems.indexer.spindexer.Spindexer;
import igknighters.subsystems.indexer.spindexer.SpindexerSim;

public class Indexer extends SubsystemBase {
    private Spindexer spindexer;
    private ExitRollers exitRollers;
    private final IndexerVisualizer visualizer = new IndexerVisualizer();

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

    public double getSpindexerRPM() {
        return spindexer.getRPM();
    }

    public double getExitRollerRPM() {
        return exitRollers.getSpeedRPM();
    }

    public void goToState(IndexerState state) {
        spindexer.goToRPM(state.getSpindexerRPM());
        exitRollers.setSpeedRPM(state.getExitRollerRPM());
    }

    public void stop() {
        spindexer.stop();
    }

    @Override
    public void periodic() {
        spindexer.periodic();
        exitRollers.periodic();
        visualizer.update(spindexer.getRPM(), exitRollers.getSpeedRPM());
    }
}
