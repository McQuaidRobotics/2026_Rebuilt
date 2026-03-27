package igknighters.commands;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.Robot;
import igknighters.util.log.Log;

public class RepulsorVisualizer {
    private static final Mechanism2d repulsor = new Mechanism2d(1, 1);
    private static final MechanismRoot2d repulse = repulsor.getRoot("repulsor", .5, .5);
    private static final MechanismLigament2d goalLigament =
            repulse.append(
                    new MechanismLigament2d(
                            "GOAL_LIGAMENT", .3, 0.0, 10, new Color8Bit(Color.kAqua)));
    private static final MechanismLigament2d repulseLigament =
            repulse.append(
                    new MechanismLigament2d(
                            "REPULSE_LIGAMENT", .3, 0.0, 10, new Color8Bit(Color.kDarkSalmon)));

    private double yGoal = 0.0;

    public RepulsorVisualizer() {
        repulsor.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("Visualizer/Repulsor Visualizer", repulsor);
    }

    public void updateYGoalForce(double yGoal) {
        this.yGoal = yGoal;
    }

    public static Color8Bit getStrengthColor(double strength) {

        if (strength <= 0.0) {
            return new Color8Bit(255, 0, 0);
        }
        double ratio = strength / 10;

        ratio = Math.min(ratio, 1.0);

        double g = 255.0 * Math.min(ratio, 1.0);
        double r = 255.0 * (1.0 - ratio);

        return new Color8Bit((int) r, (int) g, 0);
    }

    public static void update(
            double goalTheta, double repulseTheta, double goalStrength, double repulseStrength) {
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/repulse theta", repulseTheta);
            Log.log("ROBOT/Commands/repulsor/goal theta", goalTheta);
            Log.log("ROBOT/Commands/repulsor/repulse strength", repulseStrength);
            Log.log("ROBOT/Commands/repulsor/goal strength", goalStrength);
        }
        goalLigament.setAngle(Math.toDegrees(goalTheta));
        repulseLigament.setAngle(Math.toDegrees(180) + Math.toDegrees(repulseTheta));
        goalLigament.setColor(getStrengthColor(goalStrength));
        repulseLigament.setColor(getStrengthColor(repulseStrength));
    }
}
