package igknighters.subsystems.YamShooter.flywheels;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {

    FlywheelIO io;
    FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheels() {
        if (Robot.isReplay()) {
            io = new FlywheelsIOReplay();
        } else {
            io = new FlywheelsIOTalonFX(this);
        }
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        io.setVelocitySetpoint(velocity);
    }

    public Command setVelocity(AngularVelocity velocity) {
        return run(() -> io.setVelocitySetpoint(velocity))
                .withName("Flywheels.setVelocity(" + velocity.in(RPM) + ")");
    }
    ;

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        io.updateTelemetry();
        io.updateInputs(inputs);
        Logger.processInputs("SHOOTER_FLYWHEELS", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
        io.updateInputs(inputs);
        Logger.processInputs("SHOOTER_FLYWHEELS", inputs);
    }
}
