package igknighters.subsystems.climber.chainsaw;

public class ChainsawDisabled extends Chainsaw {
    @Override
    public double getPositionInches() {
        return 0;
    }

    @Override
    public void setPositionInches(double position) {}

    @Override
    public void goToInches(double inches) {}

    @Override
    public void periodic() {}

    @Override
    public void stop() {}

    @Override
    public void coast() {}
}
