package igknighters.subsystems.climber.chainsaw;

public class ChainsawDisabled extends Chainsaw {
    @Override
    double getPositionInches() {
        return 0;
    }

    @Override
    void setPositionInches(double position) {}

    @Override
    void goToInches(double inches) {}

    @Override
    void periodic() {}

    @Override
    void stop() {}

    @Override
    void coast() {}
}
