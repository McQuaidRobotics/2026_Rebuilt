package igknighters.subsystems.climber.arms;

public abstract class Arms {

    abstract void setPosition(double position);

    abstract void goToAngleDegrees(double angleDegrees);

    abstract void periodic();

    abstract double getPositionDegrees();

    abstract void stop();

    abstract void coast();
}
