package igknighters.subsystems.YamsIntake.pivot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
    PivotIO io;
    PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public IntakePivot() {
        if (igknighters.Robot.isReplay()) {
            io = new PivotIOReplay();
        } else {
            io = new PivotIOTalonFX(this);
        }
    }

    @Override
    public void periodic() {
        io.updateTelemetry();
        io.updateInputs(inputs);
        Logger.processInputs("INTAKE_PIVOT", inputs);
    }

    public Command targetAngle(Angle angle) {
        return run(() -> io.setAngleSetpoint(angle)).withName("Turret.setAngle(" + angle + ")");
    }

    public Command setAngleAndStop(Angle angle, Angle tolerance) {
        return run(() -> io.setAngleSetpoint(angle))
                .until(() -> isNear(angle, tolerance))
                .withName("IntakePivot.setAngleAndStop(" + angle + ", " + tolerance + ")");
    }

    public boolean isNear(Angle target, Angle tolerance) {
        return Rotations.of(inputs.positionRotations).isNear(target, tolerance);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }

    public void setAngleSetpoint(Angle angle) {
        io.setAngleSetpoint(angle);
    }

    public Angle getAngle() {
        return Rotations.of(inputs.positionRotations);
    }
}
