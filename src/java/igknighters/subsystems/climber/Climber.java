package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.ChainsawReal;
import igknighters.subsystems.climber.chainsaw.ChainsawSim;

public class Climber extends SubsystemBase {
    private Chainsaw chainsaw;
    private boolean usingRealSensor;

    public Climber() {
        if (Robot.isReal()) {
            chainsaw = new ChainsawReal();
            usingRealSensor = true;
        } else {
            chainsaw = new ChainsawSim();
            usingRealSensor = false;
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

    public boolean isSensorHit() {
        return chainsaw.isSensorHit();
    }

    @Override
    public void periodic() {
        chainsaw.periodic();
    }
}
