package igknighters.subsystems.elevator.mechanism;

public abstract class ElevatorMechanism {

    public abstract void setHeight(double height);

    public abstract double getHeight();

    public abstract void moveToHeight(double height);

    public abstract boolean isAt(double height, double tolerance);

    public void periodic() {}
}
