package igknighters.subsystems.climber.servos;

public abstract class Servos {
    public abstract void periodic();

    public abstract void goToAngleDegrees(double angleDegrees);
}
