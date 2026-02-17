package igknighters.subsystems.climber.chainsaw;

public class ChainsawDisabled extends Chainsaw {
    @Override
    public void goUp() {
        // Do nothing
    }

    @Override
    public void goDown() {
        // Do nothing
    }

    @Override
    public boolean isSensorHit() {
        return false;
    }

    @Override
    public boolean isUp() {
        return false;
    }

    @Override
    public void goToState(ChainsawState state) {
        
    }

    @Override
    public boolean isDown() {
        return false;
    }

    @Override
    public void periodic() {
        // Do nothing
    }
}