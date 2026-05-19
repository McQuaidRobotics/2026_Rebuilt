package igknighters.subsystems.indexer.launcherRollers;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ExitRollersDisabled extends ExitRollersBase {
    @Override
    public Command setVelocity(AngularVelocity speed) {
        return Commands.none();
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(0.0);
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity speed) {}
}
