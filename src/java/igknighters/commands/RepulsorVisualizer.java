package igknighters.commands;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.commands.RepulsorVisualizer;

public class RepulsorVisualizer {
    private final Mechanism2d repulsor = new Mechanism2d(1, 1);
    private final MechanismRoot2d repulse = repulsor.getRoot("repulsor", .5, .5);
    private final MechanismLigament2d GoalLigament = repulse.append(new MechanismLigament2d("GOAL_LIGAMENT", .3, 0.0, 10, new Color8Bit(Color.kAqua)));
    private final MechanismLigament2d repulseLigament = repulse.append(new MechanismLigament2d("REPULSE_LIGAMENT", .3, 0.0, 10, new Color8Bit(Color.kDarkSalmon)));

    private double yGoal = 0.0;

    public RepulsorVisualizer() {
        repulsor.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("Visualizer/Repulsor Visualizer", repulsor);
    }

    public void updateYGoalForce(double yGoal){
        this.yGoal = yGoal;
    }
    
    public void update(double goalTheta, double RepulseTheta, double goalStrength, double RepulseStrength) {
        GoalLigament.setAngle(Math.toDegrees(goalTheta));
        GoalLigament.setColor();
        RepulseLigament.setAngle(RepulseStrength);
        RepulseLigament.setColor();
        }
}
