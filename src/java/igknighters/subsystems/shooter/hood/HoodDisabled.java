package igknighters.subsystems.shooter.hood;

public class HoodDisabled extends Hood {
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
}
