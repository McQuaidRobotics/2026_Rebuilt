package igknighters.subsystems.YamsIntake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    PivotIO io;
    PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot() {
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

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }

    public void setAngleSetpoint(Angle angle) {
        io.setAngleSetpoint(angle);
    }

    public double getAngle() {
        return inputs.positionRotations;
    }
}
