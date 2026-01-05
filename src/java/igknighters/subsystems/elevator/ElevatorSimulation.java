package igknighters.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import igknighters.constants.SubsystemConstants;

public class ElevatorSimulation extends Elevator {
    private final ElevatorSim elevatorSim;
    private final ProfiledPIDController profiledPIDController;
    private final ElevatorVisualizer visualizer = new ElevatorVisualizer();
    private double input = 0;

    public ElevatorSimulation() {
        elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60(2),
                        SubsystemConstants.Elevator.GEAR_RATIO,
                        SubsystemConstants.Elevator.CARRIAGE_MASS_KG,
                        1,
                        SubsystemConstants.Elevator.MIN_HEIGHT_METERS,
                        SubsystemConstants.Elevator.MAX_HEIGHT_METERS,
                        true,
                        0);
        profiledPIDController =
                new ProfiledPIDController(
                        SubsystemConstants.Elevator.kP,
                        SubsystemConstants.Elevator.kI,
                        SubsystemConstants.Elevator.kD,
                        new Constraints(
                                SubsystemConstants.Elevator.MAX_SPEED_METERS_PER_SECOND,
                                SubsystemConstants.Elevator
                                        .MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
    }

    @Override
    public void moveToHeight(double height) {
        profiledPIDController.setGoal(height);
        input = profiledPIDController.calculate(elevatorSim.getPositionMeters());
    }

    @Override
    public void setHeight(double height) {
        elevatorSim.setState(height, elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public double getHeight() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public boolean isAt(double height, double tolerance) {
        return Math.abs(getHeight() - height) <= tolerance;
    }

    @Override
    public void periodic() {
        elevatorSim.setInputVoltage(input);
        elevatorSim.update(0.02);
        DogLog.log("Subsystems/Elevator/Height", elevatorSim.getPositionMeters());
        DogLog.log("Subsystems/Elevator/Velocity", elevatorSim.getVelocityMetersPerSecond());
        DogLog.log("Subsystems/Elevator/AppliedVoltage", input);
        visualizer.update(getHeight(), profiledPIDController.getSetpoint().position);
    }
}
