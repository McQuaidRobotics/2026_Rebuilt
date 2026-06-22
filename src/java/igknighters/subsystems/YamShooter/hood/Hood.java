package igknighters.subsystems.YamShooter.hood;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood() {
        if (Robot.isReplay()) {
            io = new HoodIOReplay();
        } else {
            io = new HoodIOTalonFX(this);
        }
    }

    @Override
    public void periodic() {
        io.zeroHoodCheck();
        io.updateInputs(inputs);
        Logger.processInputs("SHOOTER_HOOD", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
        io.zeroHoodCheck();
        io.updateInputs(inputs);
        Logger.processInputs("SHOOTER_HOOD", inputs);
    }

    /** Command to move the arm to a target angle. Uses run() for continuous control. */
    public Command goToAngleDontStop(Angle angle) {
        return run(() -> io.setTargetAngle(angle)).withName("Arm.setAngle(" + angle + ")");
    }

    /**
     * Command to move the arm to a target angle and finish when reached. Uses runTo() pattern - be
     * careful with default commands!
     */
    public Command goToAngleThenStop(Angle angle) {
        return run(() -> io.setTargetAngle(angle))
                .until(() -> isNear(angle, Rotations.of(0.01)))
                .withName("Arm.goToAngle(" + angle + ")");
    }

    public boolean isNear(Angle target, Angle tolerance) {
        return Rotations.of(inputs.positionRotations).isNear(target, tolerance);
    }

    /** Returns true if the arm is within tolerance of a target position using WPILib's isNear(). */
    public boolean isAt(Angle target, Angle tolerance) {
        return Rotations.of(inputs.positionRotations).isNear(target, tolerance);
    }

    /** Returns the current arm position in rotations. */
    public Angle getAngle() {
        return Rotations.of(inputs.positionRotations);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public boolean isLimitSwitchTripped() {
        return io.isLimitSwitchTripped();
    }

    public void setAngleSetpoint(Angle angle) {
        io.setTargetAngle(angle);
    }

    public void zeroAt(Angle angle) {
        io.zeroAt(angle);
    }
}
