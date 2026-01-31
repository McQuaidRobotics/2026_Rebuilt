package igknighters.subsystems.climber.servos;

public class ServosDisabled extends Servos {

    @Override
    public void periodic() {
        // Do nothing
    }

    @Override
    public void goToAngleDegrees(double angleDegrees, ServoID servoID) {
        // Do nothing
    }
}
