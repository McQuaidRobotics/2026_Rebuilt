package igknighters.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {

    private final Mechanism2d intakeMechanism = new Mechanism2d(20, 20);

    private final MechanismRoot2d intakeRoot = intakeMechanism.getRoot("PIVOT_INTAKE", 0, 0);

    private final MechanismLigament2d intakePivot =
            intakeRoot.append(
                    new MechanismLigament2d("INTAKE_PIVOT", 20, 0)); // initial angle is flat

    public IntakeVisualizer() {
        intakeMechanism.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("Visualizers/Intake/Intake-Visualizer", intakeMechanism);
        intakePivot.setColor(new Color8Bit(Color.kBeige));
    }

    public Color8Bit getColorFromRPM(double rpm) {
        if (rpm == 0) {
            return new Color8Bit(Color.kWhite);
        } else if (rpm > 0) {
            return new Color8Bit(Color.kGreen);
        } else {
            return new Color8Bit(Color.kRed);
        }
    }

    public void update(double pivotAngleDegrees, double rollerSpeedRPM) {
        intakePivot.setAngle(pivotAngleDegrees);
        intakePivot.setColor(getColorFromRPM(rollerSpeedRPM));
    }
}
