package igknighters.subsystems.YamShooter.flywheels;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FlywheelsDisabled extends Flywheels {

    @Override
    public Command setVelocity(AngularVelocity velocity) {
        return Commands.none();
    }
    @Override
    public void setVoltage(double voltage) {
        
    }
    @Override
    public void setVelocitySetpoint(AngularVelocity velocity) {
        
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(0.0);
    }
    
}
