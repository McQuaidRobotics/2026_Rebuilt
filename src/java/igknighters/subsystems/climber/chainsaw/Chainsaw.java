package igknighters.subsystems.climber.chainsaw;

public abstract class Chainsaw {

    public abstract void setPositionInches(double position);

    public abstract void goToInches(double meters);

    public abstract void periodic();

    public abstract double getPositionInches();

    public abstract void stop();

    public abstract void coast();
}
