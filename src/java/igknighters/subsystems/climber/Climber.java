package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.ChainsawReal;
import igknighters.subsystems.climber.chainsaw.ChainsawSim;

public class Climber extends SubsystemBase {
    private Chainsaw chainsaw;

    public Climber() {
        if (Robot.isReal()) {
            chainsaw = new ChainsawReal();
        } else {
            chainsaw = new ChainsawSim();
        }
    }

    public double getPositionInches() {
        return chainsaw.getPositionInches();
    }

    public void setPositionInches(double position) {
        chainsaw.setPositionInches(position);
    }

    public void goToInches(double inches) {
        chainsaw.goToInches(inches);
    }

    public boolean isAt(double targetInches, double toleranceInches) {
        return Math.abs(getPositionInches() - targetInches) <= toleranceInches;
    }

    @Override
    public void periodic() {
        chainsaw.periodic();
    }
}
