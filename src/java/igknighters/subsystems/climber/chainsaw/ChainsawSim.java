package igknighters.subsystems.climber.chainsaw;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class ChainsawSim extends Chainsaw {

    private double inputVoltage = 0.0;

    // 1 rotation = 4.5 inches = 0.1143 meters
    private static final double ROT_TO_METERS = 4.5 * Conv.INCHES_TO_METERS;
    private static final double METERS_TO_ROT = 1.0 / ROT_TO_METERS;

    private final ElevatorSim chainsawSim =
            new ElevatorSim(
                    LinearSystemId.createElevatorSystem(
                            DCMotor.getKrakenX60(2),
                            60.0,
                            0.02,
                            SubsystemConstants.kClimber.kChainsaw.GEAR_RATIO),
                    DCMotor.getKrakenX60(2),
                    SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES * Conv.INCHES_TO_METERS,
                    SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES * Conv.INCHES_TO_METERS,
                    true,
                    0.0);

    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    SubsystemConstants.kClimber.kChainsaw.kP,
                    SubsystemConstants.kClimber.kChainsaw.kI,
                    SubsystemConstants.kClimber.kChainsaw.kD,
                    new Constraints(
                            SubsystemConstants.kClimber.kChainsaw.MAX_VELOCITY_METERS_PER_SECOND,
                            SubsystemConstants.kClimber.kChainsaw
                                    .MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    private final ElevatorFeedforward feedforward =
            new ElevatorFeedforward(
                    SubsystemConstants.kClimber.kChainsaw.kS,
                    SubsystemConstants.kClimber.kChainsaw.kG,
                    SubsystemConstants.kClimber.kChainsaw.kV,
                    SubsystemConstants.kClimber.kChainsaw.kA);

    private boolean isPidControlledThisCycle = false;
    private boolean isVoltageControlledThisCycle = false;

    @Override
    public void coast() {
        inputVoltage = 0.0;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public void goToInches(double inches) {
        double goalRot =
                inches * SubsystemConstants.kClimber.kChainsaw.INCHES_TO_ROTATIONS; // inches → rotations
        profiledPIDController.setGoal(goalRot);
        isPidControlledThisCycle = true;
    }

    @Override
    public void setPositionInches(double position) {
        double rot =
                position * SubsystemConstants.kClimber.kChainsaw.INCHES_TO_ROTATIONS; // inches → rotations
        chainsawSim.setState(
                rot * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES * Conv.INCHES_TO_METERS,
                0.0); // rotations → meters
    }

    @Override
    public void stop() {
        inputVoltage = 0.0;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public boolean isSensorHit() {
        return false;
    }

    @Override
    public double getPositionInches() {
        double meters = chainsawSim.getPositionMeters();
        return (meters * METERS_TO_ROT)
                * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES; // meters → rotations → inches
    }

    @Override
    public void periodic() {

        // Convert sim meters → rotations
        double currentRot = chainsawSim.getPositionMeters() * METERS_TO_ROT;

        double goalRot = profiledPIDController.getGoal().position;

        double pidVolts = 0.0;
        double ffVolts = 0.0;
        double voltage = 0.0;

        if (isPidControlledThisCycle) {

            // PID in rotations
            pidVolts = profiledPIDController.calculate(currentRot);

            // Feedforward expects velocity in rotations/sec
            double velRotPerSec = profiledPIDController.getSetpoint().velocity;

            ffVolts = feedforward.calculate(velRotPerSec);

            voltage = pidVolts + ffVolts;
        }

        if (isVoltageControlledThisCycle) {
            voltage = inputVoltage;
        }

        voltage = MathUtil.clamp(voltage, -12.0, 12.0);

        // Logging
        DogLog.log("Subsystems/Climber/Chainsaw/SimVoltage", voltage);
        DogLog.log("Subsystems/Climber/Chainsaw/SimPositionRot", currentRot);
        DogLog.log("Subsystems/Climber/Chainsaw/GoalRot", goalRot);
        DogLog.log(
                "Subsystems/Climber/Chainsaw/PIDErrorRot",
                profiledPIDController.getPositionError());
        DogLog.log("Subsystems/Climber/Chainsaw/PIDVolts", pidVolts);
        DogLog.log("Subsystems/Climber/Chainsaw/FFVolts", ffVolts);

        // Convert rotations → meters for sim
        chainsawSim.setInputVoltage(voltage);
        chainsawSim.update(0.02);

        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
