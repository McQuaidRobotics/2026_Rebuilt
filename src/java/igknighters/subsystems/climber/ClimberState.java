package igknighters.subsystems.climber;

import igknighters.constants.SubsystemConstants;

public enum ClimberState {
    EXTENDED_NO_SERVOS(SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES, false, false),
    EXTENDED_WITH_TOP_SERVOS(SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES, false, true),
    RETRACTED_PRE_HAND_OFF(SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES, false, true),
    RETRACTED_HAND_OFF(SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES, true, true),
    EXTENDED_WITH_CLINGING(SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES, true, false),
    EXTENDED_WITH_CLINGING_DEPLOYING_SERVOS(
            SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES, true, true);

    public final double targetHeightInches;
    public final boolean stationaryClimberServosDeployed;
    public final boolean movingClimberServosDeployed;

    private ClimberState(
            double targetHeightInches,
            boolean stationaryClimberServosDeployed,
            boolean movingClimberServosDeployed) {
        this.targetHeightInches = targetHeightInches;
        this.stationaryClimberServosDeployed = stationaryClimberServosDeployed;
        this.movingClimberServosDeployed = movingClimberServosDeployed;
    }
}
