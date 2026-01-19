package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.ChainsawDisabled;
import igknighters.subsystems.climber.chainsaw.ChainsawReal;

public class Climber extends SubsystemBase {
    private Chainsaw chainsaw;

    public Climber() {
        if (Robot.isReal()) {
            chainsaw = new ChainsawReal();
        } else {
            chainsaw = new ChainsawDisabled();
        }
    }
}
