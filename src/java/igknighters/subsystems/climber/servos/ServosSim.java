package igknighters.subsystems.climber.servos;

import dev.doglog.DogLog;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class ServosSim extends Servos {
    private double angleDegrees = 0.0;
    public ServosSim() {
    }
    @Override
    public void setAngleDegrees(double angleDegrees) {
        this.angleDegrees = angleDegrees;
    }

    @Override
    public double getAngleDegrees() {
        return this.angleDegrees;
    }

    @Override
    public void periodic() {
        DogLog.log("Subsystems/Climber/Servos/Position", getAngleDegrees());
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Climber/Servos/TARGETING", angleDegrees);
        setAngleDegrees(angleDegrees);      
    }

    @Override
    public boolean isAt(double targetAngleDegrees, double toleranceDegrees) {
        return Math.abs(getAngleDegrees() - targetAngleDegrees) <= toleranceDegrees;
    }

    
}
