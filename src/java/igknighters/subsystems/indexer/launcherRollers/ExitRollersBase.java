package igknighters.subsystems.indexer.launcherRollers;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ExitRollersBase extends SubsystemBase {

    public abstract Command setVelocity(AngularVelocity speed);

    public abstract void setVelocitySetpoint(AngularVelocity speed);

    public abstract AngularVelocity getVelocity();
}
