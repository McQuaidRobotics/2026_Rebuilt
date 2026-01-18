package igknighters.subsystems.climber.chainsaw;

public abstract class Chainsaw {

    abstract void setPosition(double position);

    abstract void goToMeters(double meters);

    abstract void periodic();

    abstract double getPositionMeters();

    abstract void stop();

    abstract void coast();
}
