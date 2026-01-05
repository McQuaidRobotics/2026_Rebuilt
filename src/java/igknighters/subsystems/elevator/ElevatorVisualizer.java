package igknighters.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualizer {
    private Mechanism2d elevator;
    private MechanismLigament2d carriage, goal;

    public ElevatorVisualizer() {
        elevator = new Mechanism2d(10, 50);
        carriage =
                elevator.getRoot("Elevator", 5, 0)
                        .append(new MechanismLigament2d("Carriage", 10, 90));
        goal = elevator.getRoot("Elevator", 5, 0).append(new MechanismLigament2d("Goal", 5, 90));

        elevator.setBackgroundColor(new Color8Bit(Color.kBlack));
        carriage.setColor(new Color8Bit(Color.kBlue));
        goal.setColor(new Color8Bit(Color.kRed));
        SmartDashboard.putData("ELEVATOR VISUALIZER", elevator);
    }

    public void update(double currentHeight, double goalHeight) {
        carriage.setLength(currentHeight * 10); // scale for visualization
        goal.setLength(goalHeight * 10); // scale for visualization
    }
}
