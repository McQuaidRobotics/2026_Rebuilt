package igknighters.subsystems.climber.chainsaw;

public abstract class Chainsaw {
    public enum ChainsawState {
        BETWEEN,
        GOING_UP,
        GOING_DOWN,
        STOPPED
    }

    public abstract void goUp();

    public abstract void goDown();

    public abstract boolean isSensorHit();

    public abstract boolean isUp();

    public abstract boolean isDown();

    public abstract void goToState(ChainsawState state);

    public abstract void periodic();
}
