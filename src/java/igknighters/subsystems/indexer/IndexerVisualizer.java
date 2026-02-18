package igknighters.subsystems.indexer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IndexerVisualizer {
    private double spindexerTheta = 0.0;
    private double exitRollerTheta = 0.0;
    private final Mechanism2d indexerMechanism = new Mechanism2d(20, 20);
    private final MechanismRoot2d spindexerRoot = indexerMechanism.getRoot("INDEXER_ROOT", 5, 10);
    private final MechanismRoot2d exitRollerRoot =
            indexerMechanism.getRoot("EXIT_ROLLER_ROOT", 15, 10);
    private final MechanismLigament2d exitRollers =
            exitRollerRoot.append(new MechanismLigament2d("EXIT_ROLLERS", 10, 0));
    private final MechanismLigament2d spindexer =
            spindexerRoot.append(new MechanismLigament2d("SPINDEXER", 10, 0));

    public IndexerVisualizer() {
        // Setup Mechanism2d
        exitRollers.setColor(new Color8Bit(255, 0, 0)); // Red
        spindexer.setColor(new Color8Bit(0, 0, 255)); // Blue

        indexerMechanism.setBackgroundColor(new Color8Bit(Color.kBlack));

        SmartDashboard.putData("Visualizers/Indexer/Indexer-Visualizer", indexerMechanism);
    }

    public void update(double spindexerRPM, double exitRollerRPM) {
        spindexerTheta += spindexerRPM * 0.02;
        exitRollerTheta += exitRollerRPM * 0.02;

        spindexer.setAngle(new Rotation2d(spindexerTheta * 2 * Math.PI));
        exitRollers.setAngle(new Rotation2d(exitRollerTheta * 2 * Math.PI));
    }
}
