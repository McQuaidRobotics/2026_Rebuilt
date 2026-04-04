package igknighters.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import igknighters.Robot;
import igknighters.util.log.Log;

public class PivotSim extends Pivot {

    private Angle currentAngle;

    public PivotSim() {
        currentAngle = Degrees.of(0);
    }

    @Override
    public void setAngle(Angle angle) {
        super.degrees = angle.in(Degrees);
        currentAngle = angle;
    }

    private boolean isControlledThisCycle = false;

    @Override
    public void goToAngle(Angle angle) {
        super.targetDegrees = angle.in(Degrees);
        currentAngle = angle;
        isControlledThisCycle = true;
    }

    @Override
    public Angle getAngle() {
        return currentAngle;
    }

    @Override
    public void stop() {}

    @Override
    public void periodic() {

        if (!Robot.consts.intake().kPivot().disablePivotLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/AngleDegrees", getAngle().in(Degrees));
            Log.log("ROBOT/Subsystems/Intake/Pivot/TargetDegrees", super.targetDegrees);
        }
    }
}
