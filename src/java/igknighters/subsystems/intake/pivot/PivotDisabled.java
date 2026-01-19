package igknighters.subsystems.intake.pivot;

public class PivotDisabled extends Pivot {
    @Override
    public void goToAngleDegrees(double angleDegrees) {}

    @Override
    public void setAngleDegrees(double angleDegrees) {}

    @Override
    public double getAngleDegrees() {
        return 0;
    }

    @Override
    public void periodic() {}

    @Override
    public void stop() {}
}
