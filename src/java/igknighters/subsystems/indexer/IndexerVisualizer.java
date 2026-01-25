package igknighters.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IndexerVisualizer {
    private Mechanism2d indexer = new Mechanism2d(20, 20);

    public IndexerVisualizer() {
        SmartDashboard.putData("Visualizers/Indexer", indexer);
    }

    public void update(boolean isMoving) {
        indexer.setBackgroundColor(new Color8Bit(isMoving ? Color.kGreen : Color.kRed));
    }
}
