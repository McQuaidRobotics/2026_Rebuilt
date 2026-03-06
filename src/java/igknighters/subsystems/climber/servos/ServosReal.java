package igknighters.subsystems.climber.servos;

import edu.wpi.first.wpilibj.Servo;
import igknighters.constants.SubsystemConstants;

public class ServosReal extends Servos {

    private final Servo servo;

    public ServosReal() {
        servo = new Servo(SubsystemConstants.kClimber.kServos.SERVO_PORT_1);
    }

    private boolean deployed = false;

    @Override
    public void deploy() {
        deployed = true;
        servo.setAngle(SubsystemConstants.kClimber.kServos.MAX_ANGLE_DEGREES);
    }

    @Override
    public void retract() {
        deployed = false;
        servo.setAngle(SubsystemConstants.kClimber.kServos.MIN_ANGLE_DEGREES);
    }

    @Override
    public void periodic() {}
}
