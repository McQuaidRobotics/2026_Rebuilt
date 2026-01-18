package igknighters.subsystems.climber.chainsaw;

public abstract class Chainsaw {

    abstract void setPositionInches(double position);

    abstract void goToInches(double meters);

    abstract void periodic();

    abstract double getPositionInches();

    abstract void stop();

    abstract void coast();
}
