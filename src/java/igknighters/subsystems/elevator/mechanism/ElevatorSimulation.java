package igknighters.subsystems.elevator.mechanism;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.elevator.ElevatorVisualizer;

public class ElevatorSimulation extends ElevatorMechanism {
    private final ElevatorSim elevatorSim;
    private final ProfiledPIDController profiledPIDController;
    private boolean isCalledRepeatedly = false;
    private final ElevatorFeedforward feedforward =
            new ElevatorFeedforward(
                    SubsystemConstants.Elevator.kS,
                    SubsystemConstants.Elevator.kG,
                    SubsystemConstants.Elevator.kV,
                    SubsystemConstants.Elevator.kA);
    private final ElevatorVisualizer visualizer = new ElevatorVisualizer();
    private double input = 0;

    public ElevatorSimulation() {
        elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60(2),
                        SubsystemConstants.Elevator.GEAR_RATIO,
                        SubsystemConstants.Elevator.CARRIAGE_MASS_KG,
                        SubsystemConstants.Elevator.DRUM_RADIUS_METERS,
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
        if (height != profiledPIDController.getGoal().position) {
            DogLog.log("Subsystems/Elevator/NewGoal", height);
            // If the goal changes significantly, we might want to reset, or just let the profile
            // handle it.
            // resetting to current position prevents a jump if we are far away,
            // but the profile handles that too.
            // But if we are disabled and then enabled, we should probably re1set.
        }
        profiledPIDController.setGoal(height);

        if (height == 0) {
            DogLog.log("Subsystems/Elevator/MovingToZero", true);
        } else {
            DogLog.log("Subsystems/Elevator/MovingToZero", false);
        }
        isCalledRepeatedly = true;
    }

    @Override
    public void setHeight(double height) {
        elevatorSim.setState(height, elevatorSim.getVelocityMetersPerSecond());
        profiledPIDController.reset(height);
        profiledPIDController.setGoal(height);
    }

    @Override
    public double getHeight() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public boolean isAt(double height, double tolerance) {

        boolean atGoal = Math.abs(getHeight() - height) <= tolerance;
        DogLog.log("Subsystems/Elevator/AtGoal", atGoal);
        DogLog.log("Subsystems/Elevator/PidError", Math.abs(getHeight() - height));
        return atGoal;
    }

    @Override
    public void periodic() {
        // Calculate the next voltage based on the profile and PID
        double pidOutput = profiledPIDController.calculate(elevatorSim.getPositionMeters());

        // double pidOutput = 0.0;

        // Calculate feedforward based on the profile's setpoint velocity
        double ffOutput = feedforward.calculate(profiledPIDController.getSetpoint().velocity);
        double input;
        if (isCalledRepeatedly) {
            input = pidOutput + ffOutput;
        } else {
            input = 0;
        }

        elevatorSim.setInputVoltage(input);
        elevatorSim.update(0.02);

        DogLog.log("Subsystems/Elevator/Height", getHeight());
        DogLog.log("Subsystems/Elevator/Velocity", elevatorSim.getVelocityMetersPerSecond());
        DogLog.log("Subsystems/Elevator/AppliedVoltage", input);

        DogLog.log("Subsystems/Elevator/repeatedlyCalled", isCalledRepeatedly);

        visualizer.update(getHeight(), profiledPIDController.getGoal().position);
        isCalledRepeatedly = false;
        input = 0;
    }
}
