package igknighters.subsystems.YamShooter.flywheels;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Flywheels extends SubsystemBase {
    public abstract void setVelocitySetpoint(AngularVelocity velocity);

    public abstract Command setVelocity(AngularVelocity velocity);

    public abstract AngularVelocity getVelocity();
}
