package igknighters.subsystems.shooter.solvers;
import edu.wpi.first.math.interpolation.Interpolatable;

public record FullShooterParams(double rpm, double angle, double timeOfFlight) 
    implements Interpolatable<FullShooterParams> {

    @Override
    public FullShooterParams interpolate(FullShooterParams endValue, double t) {
        // Simple linear interpolation formula: start + (end - start) * t
        double interpolatedRpm = this.rpm + (endValue.rpm - this.rpm) * t;
        double interpolatedAngle = this.angle + (endValue.angle - this.angle) * t;
        double interpolatedTimeOfFlight = this.timeOfFlight + (endValue.timeOfFlight - this.timeOfFlight) * t;

        return new FullShooterParams(interpolatedRpm, interpolatedAngle, interpolatedTimeOfFlight);
    }
}