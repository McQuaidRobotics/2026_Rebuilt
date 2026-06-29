package igknighters.subsystems.indexer.launcherRollers;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import org.littletonrobotics.junction.Logger;

public class ExitRollers extends SubsystemBase {

    ExitRollersIO io;
    ExitRollersIOInputsAutoLogged inputs = new ExitRollersIOInputsAutoLogged();

    public ExitRollers() {
        if (Robot.isReplay()) {
            this.io = new ExitRollersIOReplay();
        } else {
            this.io = new ExitRollersIOTalonFx(this);
        }
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        io.setVelocitySetpoint(velocity);
    }

    public Command setVelocity(AngularVelocity velocity) {
        return run(() -> io.setVelocitySetpoint(velocity))
                .withName("ExitRollers.setVelocity(" + velocity.in(RPM) + ")");
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
        Logger.processInputs("INDEXER_EXIT_ROLLERS", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
