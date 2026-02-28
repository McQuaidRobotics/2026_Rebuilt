package igknighters.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;

public class PivotDisabled extends Pivot {
    @Override
    public void goToAngle(Angle angle) {}

    @Override
    public void setAngle(Angle angle) {}

    @Override
    public Angle getAngle() {
        return Degree.of(0);
    }

    @Override
    public void periodic() {}

    @Override
    public void stop() {}
}
