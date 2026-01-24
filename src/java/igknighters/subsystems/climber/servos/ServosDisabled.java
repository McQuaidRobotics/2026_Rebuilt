package igknighters.subsystems.climber.servos;

public class ServosDisabled extends Servos {
    @Override
    public void setAngleDegrees(double angleDegrees) {
        // Do nothing
    }

    @Override
    public double getAngleDegrees() {
        return 0.0;
    }

    @Override
    public void periodic() {
        // Do nothing
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        // Do nothing
    }
    @Override
    public boolean isAt(double targetAngleDegrees, double toleranceDegrees) {
        return true;
    }
    
}
