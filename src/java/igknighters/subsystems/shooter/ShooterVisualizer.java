package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

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

    private final double TURRET_WIDTH = 30;

    private final double ROBOT_LENGTH = .6;

    private final double HOOD_LENGTH = 0.2;

    private final double HOOD_WIDTH = 0.2;

    private final Color8Bit RobotColor = new Color8Bit(Color.kAliceBlue);
    private final Color8Bit frontRobot = new Color8Bit(Color.kBeige);

    private final MechanismLigament2d robotBridge =
            turret.append(
                    new MechanismLigament2d(
                            "bridge", ROBOT_LENGTH / 2, 0.0, 0.0, new Color8Bit(Color.kBlack)));
    private final MechanismLigament2d bridgeToTop =
            robotBridge.append(
                    new MechanismLigament2d(
                            "bridge-to-top",
                            ROBOT_LENGTH / 2,
                            90.0,
                            5,
                            new Color8Bit(Color.kYellow)));
    private final MechanismLigament2d topRtoTopL =
            bridgeToTop.append(
                    new MechanismLigament2d(
                            "top-to-right", ROBOT_LENGTH, 90.0, 5, new Color8Bit(Color.kAqua)));
    private final MechanismLigament2d topLToBottomL =
            topRtoTopL.append(
                    new MechanismLigament2d(
                            "topL-to-bottomL",
                            ROBOT_LENGTH,
                            90.0,
                            5,
                            new Color8Bit(Color.kViolet)));
    private final MechanismLigament2d bottomLToBottomR =
            topLToBottomL.append(
                    new MechanismLigament2d(
                            "botomL-to-right", ROBOT_LENGTH, 90.0, 5, new Color8Bit(Color.kAqua)));
    private final MechanismLigament2d bottomToBridge =
            bottomLToBottomR.append(
                    new MechanismLigament2d(
                            "bottom-to-bridge",
                            ROBOT_LENGTH / 2,
                            90.0,
                            5,
                            new Color8Bit(Color.kYellow)));

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

    private final MechanismLigament2d hoodGoalLigament =
            hood.append(
                    new MechanismLigament2d(
                            "HOOD_GOAL",
                            HOOD_LENGTH,
                            0.0,
                            HOOD_WIDTH,
                            new Color8Bit(Color.kAliceBlue)));

    public ShooterVisualizer() {
        shooter.setBackgroundColor(new Color8Bit(Color.kBlack));

        SmartDashboard.putData("Visualizers/Shooter/Shooter Visualizer", shooter);
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

    public void update(ShooterState shooterState, double targetRPM, double targetHoodAngleDegs) {
        double turretAngleDegrees = shooterState.turretAngle.in(Degrees);
        double hoodAngleDegrees = shooterState.hoodAngle.in(Degrees);
        double rpm = shooterState.flywheelSpeed.in(RPM);

        turretLigament.setAngle(turretAngleDegrees);
        hoodLigament.setAngle(hoodAngleDegrees);
        hoodGoalLigament.setAngle(targetHoodAngleDegs);
        hoodLigament.setColor(getRPMColor(rpm, targetRPM));
    }
}
