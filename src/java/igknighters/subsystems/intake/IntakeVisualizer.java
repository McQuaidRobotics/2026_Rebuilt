package igknighters.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {
    private final Mechanism2d intake = new Mechanism2d(1, 1);

    private final MechanismRoot2d pivot = intake.getRoot("pivot", 0.5, 0.5);

    private final MechanismLigament2d intakeLigament =
            pivot.append(
                    new MechanismLigament2d(
                            "bridge", 0.3, 0.0, 10, new Color8Bit(Color.kAntiqueWhite)));

    public IntakeVisualizer() {
        intake.setBackgroundColor(new Color8Bit(Color.kBlack));

        SmartDashboard.putData("Intake Visualizer", intake);
    }

    public Color8Bit getRPMColor(double rpm) {

        double ratio = rpm / 300;

        double g = 255.0 * ratio;
        double r = 255.0;

        return new Color8Bit((int) r, (int) g, 0);
    }

    public void update(double pivotAngleDegrees, double RPM) {

        intakeLigament.setAngle(pivotAngleDegrees);
        intakeLigament.setColor(getRPMColor(RPM));
    }
}
