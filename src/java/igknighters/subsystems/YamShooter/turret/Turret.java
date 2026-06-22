package igknighters.subsystems.YamShooter.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

    TurretIO io;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret() {
        if (Robot.isReplay()) {
            io = new TurretIOReplay();
        } else {
            io = new TurretIOTalonFX(this);
        }
    }

    @Override
    public void periodic() {
        io.updateTelemetry();
        io.updateInputs(inputs);
        Logger.processInputs("SHOOTER_TURRET", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
        io.updateInputs(inputs);
        Logger.processInputs("SHOOTER_TURRET", inputs);
    }

    public Command targetAngle(Angle angle) {
        return run(() -> io.setAngleSetpoint(angle)).withName("Turret.setAngle(" + angle + ")");
    }
    ;

    public Command setAngleAndStop(Angle angle, Angle tolerance) {
        return run(() -> io.setAngleSetpoint(angle))
                .until(() -> isNear(angle, tolerance))
                .withName("Turret.setAngleAndStop(" + angle + ", " + tolerance + ")");
    }
    ;

    public boolean isNear(Angle target, Angle tolerance) {
        return Rotations.of(inputs.positionRotations).isNear(target, tolerance);
    }

    public void setAngleSetpoint(Angle angle) {
        io.setAngleSetpoint(angle);
    }
    ;

    public Angle getAngle() {
        return io.getAngle();
    }
    ;

    public boolean isAt(Angle angle, Angle tolerance) {
        return isNear(angle, tolerance);
    }
    ;

    public Angle wrapAngle(Angle angle) {
        return io.wrapAngle(angle);
    }
    ;
}
