package igknighters.subsystems.shooter;

public class ShooterState {
    public final double rpm;
    public final double turretAngleRads;
    public final double hoodAngleRads;

    public ShooterState(double rpm, double turretAngleRads, double hoodAngleRads) {
        this.rpm = rpm;
        this.turretAngleRads = turretAngleRads;
        this.hoodAngleRads = hoodAngleRads;
    }

}
