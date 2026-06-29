package igknighters.subsystems.indexer.spindexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
    SpindexerIO io;
    SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer() {
        if (Robot.isReplay()) {
            this.io = new SpindexerIOReplay();
        } else {
            this.io = new SpindexerIOTalonFX(this);
        }
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        io.setVelocitySetpoint(velocity);
    }

    public Command setVelocity(AngularVelocity velocity) {
        return run(() -> io.setVelocitySetpoint(velocity))
                .withName("Spindexer.setVelocity(" + velocity.in(RPM) + ")");
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
        Logger.processInputs("INDEXER_SPINDEXER", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
