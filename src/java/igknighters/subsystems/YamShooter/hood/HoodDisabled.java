package igknighters.subsystems.YamShooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HoodDisabled extends Hood {

    @Override
    public Angle getAngle() {
        return Degrees.of(0.0);
    }

    @Override
    public Command targetAngle(Angle angle) {
        return Commands.none();
    }

    @Override
    public void zeroAt(Angle angle) {
    }

    @Override
    public void setVoltage(double voltage) {
        
    }

    @Override
    public Command setAngleAndStop(Angle angle, Angle tolerance) {
        return Commands.none();
    }

    @Override
    public boolean isLimitSwitchTripped() {
        return false;
    }

    @Override
    public void setAngleSetpoint(Angle angle) {
        
    }
    @Override
    public boolean isAt(Angle angle, Angle tolerance) {
        return true;
    }
    
}
