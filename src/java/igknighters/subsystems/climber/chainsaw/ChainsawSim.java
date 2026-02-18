package igknighters.subsystems.climber.chainsaw;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

    @Override
    public void goToState(igknighters.subsystems.climber.chainsaw.Chainsaw.ChainsawState state) {
        this.state = ChainsawState.valueOf(state.name());
    }

    private ChainsawState state = ChainsawState.STOPPED;

    @Override
    public void goDown() {
        state = ChainsawState.GOING_DOWN;
    }

    @Override
    public void goUp() {

        state = ChainsawState.GOING_UP;
    }

    @Override
    public boolean isDown() {
        return chainsawSim.getPositionMeters()
                <= SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES * Conv.INCHES_TO_METERS
                        + 0.01;
    }

    @Override
    public boolean isUp() {
        return chainsawSim.getPositionMeters()
                >= SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES * Conv.INCHES_TO_METERS
                        - 0.01;
    }

    @Override
    public boolean isMiddle() {
        double currentHeightInches = chainsawSim.getPositionMeters() * Conv.METERS_TO_INCHES;
        return Math.abs(
                        currentHeightInches
                                - SubsystemConstants.kClimber.kChainsaw.MIDDLE_HEIGHT_INCHES)
                <= 0.5;
    }

    @Override
    public boolean isSensorHit() {
        return false;
    }

    @Override
    public void periodic() {

        double voltage = 0.0;
        if (state == ChainsawState.GOING_UP) {
            if (isUp()) {
                state = ChainsawState.STOPPED;
                voltage = 0.0;
            } else {
                voltage = 6.0; // Simulated 6V up
            }
        } else if (state == ChainsawState.GOING_DOWN) {
            if (isDown()) {
                state = ChainsawState.STOPPED;
                voltage = 0.0; // Simulated -6V down
            } else {
                voltage = -6.0;
            }
        } else if (state == ChainsawState.GOING_TO_MIDDLE) {
            if (isMiddle()) {
                state = ChainsawState.STOPPED;
                voltage = 0.0;
            } else {
                double currentHeightInches =
                        chainsawSim.getPositionMeters() * Conv.METERS_TO_INCHES;
                if (currentHeightInches
                        < SubsystemConstants.kClimber.kChainsaw.MIDDLE_HEIGHT_INCHES) {
                    voltage = 6.0; // go up
                } else {
                    voltage = -6.0; // go down
                }
            }
        }

        // Convert sim meters → rotations
        double currentRot = chainsawSim.getPositionMeters() * METERS_TO_ROT;

        // Logging
        DogLog.log("Subsystems/Climber/Chainsaw/SimVoltage", voltage);
        DogLog.log("Subsystems/Climber/Chainsaw/SimPositionRot", currentRot);
        DogLog.log(
                "Subsystems/Climber/Inches",
                currentRot * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES);
        DogLog.log("Subsystems/Climber/Is Up", isUp());
        DogLog.log("Subsystems/Climber/Is Middle", isMiddle());
        DogLog.log("Subsystems/Climber/Is Down", isDown());
        DogLog.log("Subsystems/Climber/State", state.toString());

        // Update sim
        chainsawSim.setInputVoltage(voltage);
        chainsawSim.update(0.02);
    }
}
