package igknighters.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.indexer.spindexer.Spindexer;
import igknighters.subsystems.indexer.spindexer.SpindexerReal;
import igknighters.subsystems.indexer.spindexer.SpindexerSim;

public class Indexer extends SubsystemBase {
    private Spindexer spindexer;

    public Indexer() {
        if (Robot.isReal()) {
            spindexer = new SpindexerReal();
        } else {
            spindexer = new SpindexerSim();
        }
    }

    public void setRPM(double RPM) {
        spindexer.goToRPM(RPM);
    }

    public double getRPM() {
        return spindexer.getRPM();
    }

    public void stop() {
        spindexer.stop();
    }
}
