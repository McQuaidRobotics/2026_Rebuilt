package igknighters.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.intake.pivot.Pivot;
import igknighters.subsystems.intake.pivot.PivotReal;
import igknighters.subsystems.intake.pivot.PivotSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersReal;
import igknighters.subsystems.intake.rollers.RollersSim;
import igknighters.util.log.Log;

public class Intake extends SubsystemBase {
    private final Pivot pivot;
    private final Rollers rollers;
    private final IntakeVisualizer visualizer;

    public Intake() {
        if (Robot.isReal()) {
            pivot = new PivotReal();
            rollers = new RollersReal();
        } else {
            pivot = new PivotSim();
            rollers = new RollersSim();
        }
        visualizer = new IntakeVisualizer();
    }

    public void setRollerSpeed(AngularVelocity velo) {
        rollers.goToSpeed(velo);
    }

    public void goTo(Angle angle, AngularVelocity speedRPM) {
        pivot.goToAngle(angle);
        rollers.goToSpeed(speedRPM);
    }

    public void goTo(IntakeState state) {
        goTo(state.getPivotAngle(), state.getRollerSpeed());
    }

    public void goTo(boolean toggledState) {
        if (toggledState) {
            goTo(IntakeState.Stowed.getPivotAngle(), IntakeState.Stowed.getRollerSpeed());
        } else {
            goTo(IntakeState.Intake.getPivotAngle(), IntakeState.Intake.getRollerSpeed());
        }
    }

    public Angle getPivotAngle() {
        return pivot.getAngle();
    }

    public AngularVelocity getRollerSpeedRPM() {
        return rollers.getSpeed();
    }

    public void setPivotDegrees(double degrees) {
        pivot.setAngle(Degrees.of(degrees));
    }

    public void stop() {
        pivot.stop();
        rollers.stop();
    }

    public boolean isAt(
            Angle angleDegrees,
            AngularVelocity speedRPM,
            Angle angleTolerance,
            AngularVelocity speedTolerance) {
        boolean isAtAngle =
                Math.abs(pivot.getAngle().in(Degrees) - angleDegrees.in(Degrees))
                        < angleTolerance.in(Degrees);
        boolean isAtSpeed =
                Math.abs(rollers.getSpeed().in(RPM) - speedRPM.in(RPM)) < speedTolerance.in(RPM);

        if (!Robot.consts.intake().kPivot().disablePivotLogs()) {

            Log.log("ROBOT/Subsystems/Intake/AT STATE/Is At Speed", isAtSpeed);
            Log.log("ROBOT/Subsystems/Intake/AT STATE/Is At Angle", isAtAngle);
            Log.log(
                    "Subsystems/Intake/AT STATE/DELTA THETA",
                    pivot.getAngle().in(Degrees) - angleDegrees.in(Degrees));
            Log.log(
                    "Subsystems/Intake/AT STATE/DELTA RPM",
                    rollers.getSpeed().in(RPM) - speedRPM.in(RPM));
        }
        return isAtAngle && isAtSpeed;
    }

    @Override
    public void periodic() {
        pivot.periodic();
        rollers.periodic();
        if (!Robot.isReal()) {
            visualizer.update(pivot.getAngle().in(Degrees), rollers.getSpeed().in(RPM));
        }
    }
}
