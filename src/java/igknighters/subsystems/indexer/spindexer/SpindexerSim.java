package igknighters.subsystems.indexer.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIndexer.kSpindexer;
import igknighters.util.log.Log;

public class SpindexerSim extends Spindexer {
    private double inputVoltage = 0.0;

    private final FlywheelSim indexerSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.kIndexer.kSpindexer.MOMENT_OF_INERTIA_KG_M2,
                            SubsystemConstants.kIndexer.kSpindexer.GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    SubsystemConstants.kIndexer.kSpindexer.kI,
                    SubsystemConstants.kIndexer.kSpindexer.kD,
                    new Constraints(
                            SubsystemConstants.kIndexer.kSpindexer.MAX_SPEED_RPM,
                            SubsystemConstants.kIndexer.kSpindexer.MAX_ACCELERATION_RPM));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SubsystemConstants.kIndexer.kSpindexer.kS,
                    SubsystemConstants.kIndexer.kSpindexer.kV,
                    SubsystemConstants.kIndexer.kSpindexer.kA);
    private boolean isPidControlledThisCycle = false;
    private boolean isVoltageControlledThisCycle = false;

    @Override
    public void goToRPM(double RPM) {
        profiledPIDController.setGoal(RPM / 60);
        isPidControlledThisCycle = true;
    }

    @Override
    public void stop() {
        inputVoltage = 0.0;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public double getRPM() {
        return indexerSim.getAngularVelocityRPM();
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

            ffOutput = kSpindexer.kS + kSpindexer.kV * goalRPS;

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
        if (!Robot.consts.indexer().kSpindexer().disableSpindexerLogs()) {
            Log.log("ROBOT/Subsystems/Indexer/Spindexer/SimVoltage", voltage);
            Log.log("ROBOT/Subsystems/Indexer/Spindexer/SimSpeedRPM", currentRPM);
            Log.log("ROBOT/Subsystems/Indexer/Spindexer/GoalSpeedRPM", goalRPS * 60.0);
            Log.log(
                    "Subsystems/Indexer/Spindexer/PIDOutputRPM",
                    profiledPIDController.getPositionError());
            Log.log("ROBOT/Subsystems/Indexer/Spindexer/PIDVolts", pidOutput);
            Log.log("ROBOT/Subsystems/Indexer/Spindexer/FFVolts", ffOutput);
        }

        // Apply to sim
        indexerSim.setInputVoltage(voltage);
        indexerSim.update(0.02);

        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
