package igknighters.subsystems.climber.chainsaw;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.Conv;

public class ChainsawSim extends Chainsaw{
    private double inputVoltage = 0.0;

    private final ElevatorSim indexerSim =
            new ElevatorSim(
                    LinearSystemId.createElevatorSystem(
                            DCMotor.getKrakenX60(2),
                            60.0,
                            0.03),
                    DCMotor.getKrakenX60(2),
                    SubsystemConstants.kClimber.MIN_HEIGHT_INCHES * Conv.INCHES_TO_METERS,
                    SubsystemConstants.kClimber.MAX_HEIGHT_INCHES * Conv.INCHES_TO_METERS,
                    true,
                    0.0);
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    SubsystemConstants.kClimber.kI,
                    SubsystemConstants.kClimber.kD,
                    new Constraints(
                            SubsystemConstants.kClimber.MAX_VELOCITY_METERS_PER_SECOND,
                            SubsystemConstants.kClimber.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SubsystemConstants.kClimber.kS,
                    SubsystemConstants.kClimber.kV,
                    SubsystemConstants.kClimber.kA);
    private boolean isPidControlledThisCycle = false;
    private boolean isVoltageControlledThisCycle = false;

    @Override
    public void goToInches(double inches) {
        profiledPIDController.setGoal(inches / SubsystemConstants.kClimber.INCHES_TO_ROTATIONS);
        isPidControlledThisCycle = true;
    }

    @Override
    public void stop() {
        inputVoltage = 0.0;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public double getPositionInches() {
        return indexerSim.getPositionMeters() / SubsystemConstants.kClimber.INCHES_TO_ROTATIONS;
    }

    @Override
    public void periodic() {
        double currentRPM = indexerSim.getAngularVelocityRPM();
        double goalRPS = profiledPIDController.getGoal().position;
        double pidOutput = 0.0;
        double ffOutput = 0.0;

        double voltage = 0.0;

        if (isPidControlledThisCycle) {

            // Feedforward in volts

            ffOutput = kClimber.kS + kClimber.kV * goalRPS;

            // PID output is in RPM, convert to volts with a small gain
            // Tune this value (start around 0.001)
            double kRPM_to_volts = 0.002;

            double pidRPM = profiledPIDController.calculate(currentRPM);
            pidOutput = pidRPM * kRPM_to_volts;

            voltage = pidOutput + ffOutput;
        }

        if (isVoltageControlledThisCycle) {
            voltage = inputVoltage;
        }

        // Clamp to real motor limits
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);

        // Logging
        DogLog.log("Subsystems/Indexer/Spindexer/SimVoltage", voltage);
        DogLog.log("Subsystems/Indexer/Spindexer/SimSpeedRPM", currentRPM);
        DogLog.log("Subsystems/Indexer/Spindexer/GoalSpeedRPM", goalRPS * 60.0);
        DogLog.log(
                "Subsystems/Indexer/Spindexer/PIDOutputRPM",
                profiledPIDController.getPositionError());
        DogLog.log("Subsystems/Indexer/Spindexer/PIDVolts", pidOutput);
        DogLog.log("Subsystems/Indexer/Spindexer/FFVolts", ffOutput);

        // Apply to sim
        indexerSim.setInputVoltage(voltage);
        indexerSim.update(0.02);

        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
