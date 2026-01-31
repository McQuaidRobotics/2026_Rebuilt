package igknighters.subsystems.climber.servos;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import igknighters.constants.SubsystemConstants;

public class ServosReal extends Servos {

    private final Servo UpperServo1;
    private final Servo UpperServo2;
    private final Servo LowerServo1;
    private final Servo LowerServo2;

    public ServosReal() {
        UpperServo1 = new Servo(SubsystemConstants.kClimber.kServos.SERVO_PORT_1);
        UpperServo2 = new Servo(SubsystemConstants.kClimber.kServos.SERVO_PORT_2);
        LowerServo1 = new Servo(SubsystemConstants.kClimber.kServos.SERVO_PORT_3);
        LowerServo2 = new Servo(SubsystemConstants.kClimber.kServos.SERVO_PORT_4);

        // we still need to define the constants

    }

    double desiredAngleUpperServos = 0.0;
    double desiredAngleLowerServos = 0.0;

    @Override
    public void goToAngleDegrees(double angleDegrees, ServoID servoID) {
        if (servoID == ServoID.MOVING_SERVOS) {
            desiredAngleUpperServos =
                    MathUtil.clamp(
                            angleDegrees,
                            SubsystemConstants.kClimber.kServos.MIN_ANGLE_DEGREES,
                            SubsystemConstants.kClimber.kServos.MAX_ANGLE_DEGREES);
            double servoPosition = desiredAngleUpperServos / 180.0; // Normalize to [0.0, 1.0]
            UpperServo1.set(servoPosition);
            UpperServo2.set(servoPosition);
        } else if (servoID == ServoID.FIXED_SERVOS) {
            desiredAngleLowerServos =
                    MathUtil.clamp(
                            angleDegrees,
                            SubsystemConstants.kClimber.kServos.MIN_ANGLE_DEGREES,
                            SubsystemConstants.kClimber.kServos.MAX_ANGLE_DEGREES);
            double servoPosition = desiredAngleLowerServos / 180.0; // Normalize to [0.0, 1.0]
            LowerServo1.set(servoPosition);
            LowerServo2.set(
                    servoPosition); // potentailly need to be reversed eg 180 - angleDegrees,
            // angleDegrees
        }
    }

    @Override
    public void periodic() {
        DogLog.log("Subsystems/Climber/Servos/Position", desiredAngleUpperServos);
        // the angle needs to be set periodically, or the servo will get angry and turn off after
        // 100ms
        // silly silly why dont you go find a source for that
    }
}
