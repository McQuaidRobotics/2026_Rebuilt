package igknighters.subsystems.YamsIntake.rollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {

    IntakeRollersIO io;

    IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    public IntakeRollers() {
        if (Robot.isReplay()) {
            io = new IntakeRollersIOReplay();
        } else {
            io = new IntakeRollersIOTalonFX(this);
        }
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        io.setVelocitySetpoint(velocity);
    }

    public Command targetVelocity(AngularVelocity velocity) {
        return run(() -> io.setVelocitySetpoint(velocity))
                .withName("INTAKE_ROLLERS.setVelocity(" + velocity.in(RPM) + ")");
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public boolean isNear(AngularVelocity target, AngularVelocity tolerance) {
        return RotationsPerSecond.of(inputs.velocityRotationsPerSec).isNear(target, tolerance);
    }

    @Override
    public void periodic() {
        io.updateTelemetry();
        io.updateInputs(inputs);
        Logger.processInputs("INTAKE_ROLLERS", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
