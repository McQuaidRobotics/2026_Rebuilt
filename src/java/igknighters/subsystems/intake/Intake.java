package igknighters.subsystems.intake;

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
    private final IntakeVisualizer visualizer = new IntakeVisualizer();

    public Intake() {
        if (Robot.isReal()) {
            pivot = new PivotDisabled();
            rollers = new RollersDisabled();
        } else {
            pivot = new PivotSim();
            rollers = new RollersSim();
        }
    }

    public void goTo(double angleDegrees, double speedRPM) {
        pivot.goToAngleDegrees(angleDegrees);
        rollers.goToSpeedRPM(speedRPM);
    }

    public void goTo(IntakeState state) {
        goTo(state.pivotDegrees, state.rollerSpeedRPM);
    }

    public void stop() {
        pivot.stop();
        rollers.stop();
    }

    public boolean isAt(
            double angleDegrees, double speedRPM, double angleTolerance, double speedTolerance) {
        return Math.abs(pivot.getAngleDegrees() - angleDegrees) < angleTolerance
                && Math.abs(rollers.getSpeedRPM() - speedRPM) < speedTolerance;
    }

    @Override
    public void periodic() {
        pivot.periodic();
        rollers.periodic();
        visualizer.update(pivot.getAngleDegrees(), rollers.getSpeedRPM());
    }
}
