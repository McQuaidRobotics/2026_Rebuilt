package igknighters.subsystems.elevator;

import igknighters.Robot;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.elevator.mechanism.ElevatorDisabled;
import igknighters.subsystems.elevator.mechanism.ElevatorMechanism;
import igknighters.subsystems.elevator.mechanism.ElevatorSimulation;

public class Elevator implements ExclusiveSubsystem {
    private final ElevatorMechanism elevator;

    public Elevator() {
        if (Robot.isReal()) {
            elevator = new ElevatorDisabled();
        } else {
            elevator = new ElevatorSimulation();
        }
    }

    public void setHeight(double height) {
        elevator.setHeight(height);
    }

    public double getHeight() {
        return elevator.getHeight();
    }

    public void moveToHeight(double height) {
        elevator.moveToHeight(height);
    }

    public boolean isAt(double height, double tolerance) {
        return elevator.isAt(height, tolerance);
    }

    @Override
    public void periodic() {
        elevator.periodic();
    }
}
