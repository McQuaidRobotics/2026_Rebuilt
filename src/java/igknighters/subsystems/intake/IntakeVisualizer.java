package igknighters.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {
    private final Mechanism2d intake = new Mechanism2d(1, 1);

    private final MechanismRoot2d pivot = intake.getRoot("pivot", 0.5, 0.0);

    private final double PIVOT_LENGTH = 0.3;

    private final double PIVOT_WIDTH = 30;

    private final double ROBOT_LENGTH = .6;

    private final Color8Bit RobotColor = new Color8Bit(Color.kAliceBlue);
    private final Color8Bit frontRobot = new Color8Bit(Color.kBeige);

    private final MechanismLigament2d robotBridge =
            pivot.append(
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

    private final MechanismLigament2d pivotLigament =
            pivot.append(
                    new MechanismLigament2d(
                            "PIVOT_LIGAMENT",
                            PIVOT_LENGTH,
                            0.0,
                            PIVOT_WIDTH,
                            new Color8Bit(Color.kRed)));

    public IntakeVisualizer() {
        intake.setBackgroundColor(new Color8Bit(Color.kBlack));

        SmartDashboard.putData("Intake Visualizer", intake);
    }

    // public Color8Bit getRPMColor(double rpm) {

    //     if (targetRPM <= 0.0) {
    //         return new Color8Bit(255, 0, 0);
    //     }
    //     double ratio = rpm / targetRPM;

    //     ratio = Math.min(ratio, 1.0);

    //     double g = 255.0 * Math.min(ratio, 1.0);
    //     double r = 255.0 * (1.0 - ratio);

    //     return new Color8Bit((int) r, (int) g, 0);
    // }

    public void update(double pivotAngleDegrees, double RPM) {

        pivotLigament.setAngle(pivotAngleDegrees);
        // pivotLigament.setColor(getRPMColor(rpm));
    }
}
