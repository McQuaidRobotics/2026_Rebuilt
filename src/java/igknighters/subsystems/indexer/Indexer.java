package igknighters.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.indexer.spindexer.Spindexer;
import igknighters.subsystems.indexer.spindexer.SpindexerReal;
import igknighters.subsystems.indexer.spindexer.SpindexerSim;

public class Indexer extends SubsystemBase {
    private Spindexer spindexer;
    private IndexerVisualizer visualizer = new IndexerVisualizer();

    public Indexer() {
        if (Robot.isReal()) {
            spindexer = new SpindexerReal();
        } else {
            spindexer = new SpindexerSim();
        }
    }

    public void setRPM(double RPM) {
        DogLog.log("Subsystems/Spindexer/TARGET RPM", RPM);
        spindexer.goToRPM(RPM);
    }

    public double getRPM() {
        DogLog.log("Subsystems/Spindexer/ACTUAL RPM", spindexer.getRPM());
        return spindexer.getRPM();
    }

    public void stop() {
        spindexer.stop();
    }

    @Override
    public void periodic() {
        spindexer.periodic();
        if (spindexer.getRPM() != 0) {
            visualizer.update(true);
        } else {
            visualizer.update(false);
        }
    }
}
