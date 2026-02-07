package igknighters.subsystems.climber.servos;

public abstract class Servos {

    public static enum ServoID {
        MOVING_SERVOS,
        FIXED_SERVOS
    }

    public abstract void periodic();

    public abstract void goToAngleDegrees(double angleDegrees, ServoID servoID);

    public abstract void goToAngleDegrees(boolean deployed, ServoID servoID);
}
