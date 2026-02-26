package igknighters.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.intake.pivot.Pivot;
import igknighters.subsystems.intake.pivot.PivotDisabled;
import igknighters.subsystems.intake.pivot.PivotSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersDisabled;
import igknighters.subsystems.intake.rollers.RollersSim;

public class Intake extends SubsystemBase {
    private final Pivot pivot;
    private final Rollers rollers;
    private final IntakeVisualizer visualizer;

    public Intake() {
        if (Robot.isReal()) {
            pivot = new PivotDisabled();
            rollers = new RollersDisabled();
        } else {
            pivot = new PivotSim();
            rollers = new RollersSim();
        }
        visualizer = new IntakeVisualizer();
    }

    public void goTo(Angle angle, AngularVelocity speedRPM) {
        pivot.goToAngleDegrees(angle);
        rollers.goToSpeed(speedRPM);
    }

    public void goTo(IntakeState state) {
        goTo(state.pivotDegrees, state.rollerSpeedRPM);
    }

    public double getPivotAngleDegrees() {
        return pivot.getAngleDegrees();
    }

    public double getRollerSpeedRPM() {
        return rollers.getSpeedRPM();
    }

    public void setPivotDegrees(double degrees) {
        pivot.setAngleDegrees(Degrees.of(degrees));
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
        return Math.abs(pivot.getAngleDegrees() - angleDegrees.in(Degrees))
                        < angleTolerance.in(Degrees)
                && Math.abs(rollers.getSpeedRPM() - speedRPM.in(RPM)) < speedTolerance.in(RPM);
    }

    @Override
    public void periodic() {
        pivot.periodic();
        rollers.periodic();
        visualizer.update(pivot.getAngleDegrees(), rollers.getSpeedRPM());
    }
}
