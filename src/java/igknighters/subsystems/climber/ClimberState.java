package igknighters.subsystems.climber;

import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.Chainsaw.ChainsawState;

public enum ClimberState {
    STOW(ChainsawState.GOING_DOWN, false),
    CLIMB_PREP(ChainsawState.GOING_UP, true),
    PULL_UP(ChainsawState.GOING_UP, true);
    

    public final ChainsawState chainsawState;
    public final boolean stationaryClimberServosDeployed;

    private ClimberState(
            Chainsaw.ChainsawState chainsawState,
            boolean stationaryClimberServosDeployed) {
        this.chainsawState = chainsawState;
        this.stationaryClimberServosDeployed = stationaryClimberServosDeployed;
    }
}
