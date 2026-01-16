package igknighters.subsystems.intake;

import igknighters.Robot;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.intake.pivot.Pivot;
import igknighters.subsystems.intake.pivot.PivotReal;
import igknighters.subsystems.intake.pivot.PivotSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersReal;
import igknighters.subsystems.intake.rollers.RollersSim;

public class Intake implements ExclusiveSubsystem {
    private final Pivot pivot;
    private final Rollers rollers;

    public Intake() {
        if (Robot.isReal()) {
            pivot = new PivotReal();
            rollers = new RollersReal();
        } else {
            pivot = new PivotSim();
            rollers = new RollersSim();
        }
    }
}
