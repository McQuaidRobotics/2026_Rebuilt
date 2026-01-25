package igknighters.subsystems.climber;

public class ClimberState {
    public double positionInches;
    public double upperServoAngleDegrees;
    public double lowerServoAngleDegrees;

    public ClimberState(
            double positionInches, double upperServoAngleDegrees, double lowerServoAngleDegrees) {
        this.positionInches = positionInches;
        this.upperServoAngleDegrees = upperServoAngleDegrees;
        this.lowerServoAngleDegrees = lowerServoAngleDegrees;
    }
}
