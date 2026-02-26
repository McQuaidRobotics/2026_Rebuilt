package igknighters.subsystems.intake.pivot;

import edu.wpi.first.units.measure.Angle;

public class PivotDisabled extends Pivot {
    @Override
    public void goToAngleDegrees(Angle angle) {}

    @Override
    public void setAngleDegrees(Angle angle) {}

    @Override
    public double getAngleDegrees() {
        return 0;
    }

    @Override
    public void periodic() {}

    @Override
    public void stop() {}
}
