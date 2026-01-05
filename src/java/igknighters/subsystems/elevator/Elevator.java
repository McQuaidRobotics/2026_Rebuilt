package igknighters.subsystems.elevator;

import igknighters.subsystems.Subsystems.ExclusiveSubsystem;

public abstract class Elevator implements ExclusiveSubsystem {

    public abstract void setHeight(double height);

    public abstract double getHeight();

    public abstract void moveToHeight(double height);

    public abstract boolean isAt(double height, double tolerance);

    public void periodic() {}
}
