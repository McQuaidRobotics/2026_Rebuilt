package igknighters.subsystems.climber.servos;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import igknighters.constants.SubsystemConstants;

public class ServosSim extends Servos {
    private double upperAngleDegrees = 0.0;
    private double lowerAngleDegrees = 0.0;

    public ServosSim() {}

    @Override
    public void periodic() {
        DogLog.log("Subsystems/Climber/Servos/Upper Servo Position", upperAngleDegrees);
        DogLog.log("Subsystems/Climber/Servos/Lower Servo Position", lowerAngleDegrees);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees, ServoID servoID) {
        if (servoID == ServoID.MOVING_SERVOS) {
            upperAngleDegrees =
                    MathUtil.clamp(
                            angleDegrees,
                            SubsystemConstants.kClimber.kServos.MIN_ANGLE_DEGREES,
                            SubsystemConstants.kClimber.kServos.MAX_ANGLE_DEGREES);
        } else if (servoID == ServoID.FIXED_SERVOS) {
            lowerAngleDegrees =
                    MathUtil.clamp(
                            angleDegrees,
                            SubsystemConstants.kClimber.kServos.MIN_ANGLE_DEGREES,
                            SubsystemConstants.kClimber.kServos.MAX_ANGLE_DEGREES);
        }
    }
}
