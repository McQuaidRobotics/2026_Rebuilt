package igknighters.subsystems.climber.servos;

public abstract class Servos {
    public abstract void setAngleDegrees(double angleDegrees);
    
    public abstract double getAngleDegrees();
    
    public abstract void periodic();
    
    public abstract void goToAngleDegrees(double angleDegrees);

    public abstract boolean isAt(double targetAngleDegrees, double toleranceDegrees);

}
