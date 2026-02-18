package igknighters.subsystems.climber;

import igknighters.subsystems.climber.chainsaw.Chainsaw.ChainsawState;

public enum ClimberState {
    STOW(ChainsawState.GOING_DOWN, false),
    CLIMB_PREP(ChainsawState.GOING_UP, false),
    LATCH_ON(
            ChainsawState.GOING_UP,
            true), // Latching on means chainsaw goes up and servo is deployed
    PULL_UP(ChainsawState.GOING_DOWN, true); // Pulling up means chainsaw goes down

    public final ChainsawState chainsawState;
    public final boolean servoDeployed;

    private ClimberState(ChainsawState chainsawState, boolean servoDeployed) {
        this.chainsawState = chainsawState;
        this.servoDeployed = servoDeployed;
    }
}
