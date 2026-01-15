package igknighters.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ShooterVisualizer {
    private final Mechanism2d shooter = new Mechanism2d(1, 1);

    private final MechanismRoot2d hood = shooter.getRoot("Shooter", 0.5, 0.0);

    private final MechanismRoot2d turret = shooter.getRoot("TURRET", .5, .7);

    private final double TURRET_LENGTH = 0.3;

    private final double TURRET_WIDTH = 0.3;

    private final double HOOD_LENGTH = 0.2;

    private final double HOOD_WIDTH = 0.1;

    private final MechanismLigament2d turretLigament =
            turret.append(
                    new MechanismLigament2d(
                            "TURRET_LIGAMENT",
                            TURRET_LENGTH,
                            0.0,
                            TURRET_WIDTH,
                            new Color8Bit(Color.kRed)));

    private final MechanismLigament2d hoodLigament =
            hood.append(
                    new MechanismLigament2d(
                            "HOOD_LIGAMENT",
                            HOOD_LENGTH,
                            0.0,
                            HOOD_WIDTH,
                            new Color8Bit(Color.kGreen)));

    public ShooterVisualizer() {
        shooter.setBackgroundColor(new Color8Bit(Color.kBlack));

        SmartDashboard.putData("Shooter", shooter);
    }

    public Color8Bit getRPMColor(double rpm, double targetRPM) {

        if (targetRPM <= 0.0) {
            return new Color8Bit(255, 0, 0);
        }
        double ratio = rpm / targetRPM;

        ratio = Math.min(ratio, 1.0);

        double g = 255.0 * Math.min(ratio, 1.0);
        double r = 255.0 * (1.0 - ratio);

        return new Color8Bit((int) r, (int) g, 0);
    }

    public void update(ShooterState shooterState, double targetRPM) {
        double turretAngleDegrees = Math.toDegrees(shooterState.turretAngleRads);
        double hoodAngleDegrees = Math.toDegrees(shooterState.hoodAngleRads);
        double rpm = shooterState.rpm;

        turretLigament.setAngle(turretAngleDegrees);
        hoodLigament.setAngle(hoodAngleDegrees);
        hoodLigament.setColor(getRPMColor(rpm, targetRPM));
    }
}
