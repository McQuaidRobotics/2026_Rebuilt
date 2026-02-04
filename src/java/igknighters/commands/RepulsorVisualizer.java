package igknighters.commands;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.shooter.ShooterState;

public class RepulsorVisualizer {
    private final Mechanism2d repulsor = new Mechanism2d(1, 1);

    public RepulsorVisualizer() {
        repulsor.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("Visualizer/Repulsor Visualizer", repulsor);
    }

    public void update(ShooterState shooterState, double targetRPM) {
        double turretAngleDegrees = Math.toDegrees(shooterState.turretAngleRads);
        double hoodAngleDegrees = Math.toDegrees(shooterState.hoodAngleRads);
        double rpm = shooterState.rpm;

        // turretLigament.setAngle(turretAngleDegrees);
        // hoodLigament.setAngle(hoodAngleDegrees);
        // hoodLigament.setColor(getRPMColor(rpm, targetRPM));
    }
}
