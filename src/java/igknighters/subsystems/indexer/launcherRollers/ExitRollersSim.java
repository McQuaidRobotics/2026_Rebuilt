package igknighters.subsystems.indexer.launcherRollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIndexer.kExitRollers;
import igknighters.util.log.Log;

public class ExitRollersSim extends ExitRollers {
    private double inputVoltage = 0.0;

    private final FlywheelSim leaderflywheelSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.kIndexer.kExitRollers.MOMENT_OF_INERTIA_KG_M2,
                            SubsystemConstants.kIndexer.kExitRollers.GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    SubsystemConstants.kIndexer.kExitRollers.kI,
                    SubsystemConstants.kIndexer.kExitRollers.kD,
                    new Constraints(
                            SubsystemConstants.kIndexer.kExitRollers.MAX_SPEED_RPM,
                            SubsystemConstants.kIndexer.kExitRollers.MAX_ACCELERATION_RPM));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SubsystemConstants.kIndexer.kExitRollers.kS,
                    SubsystemConstants.kIndexer.kExitRollers.kV,
                    SubsystemConstants.kIndexer.kExitRollers.kA);
    private boolean isPidControlledThisCycle = false;
    private boolean isVoltageControlledThisCycle = false;

    @Override
    public boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        double currentRPM = getSpeedRPM();
        return Math.abs(currentRPM - targetRPM) <= toleranceRPM;
    }

    @Override
    public void setSpeedRPM(double speedRPM) {
        profiledPIDController.setGoal(speedRPM);
        isPidControlledThisCycle = true;
    }

    @Override
    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public double getSpeedRPM() {
        return leaderflywheelSim.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        double currentRPM = leaderflywheelSim.getAngularVelocityRPM();
        double goalRPM = profiledPIDController.getGoal().position;
        double pidOutput = 0.0;
        double ffOutput = 0.0;

        double voltage = 0.0;

        if (isPidControlledThisCycle) {

            // Feedforward in volts

            double goalRPS = goalRPM / 60.0;

            ffOutput = kExitRollers.kS + kExitRollers.kV * goalRPS;

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
        if (!Robot.consts.indexer().kExitRollers().disableExitRollersLogs()) {
            Log.log("ROBOT/Subsystems/Indexer/ExitRollers/SimVoltage", voltage);
            Log.log("ROBOT/Subsystems/Indexer/ExitRollers/SimSpeedRPM", currentRPM);
            Log.log("ROBOT/Subsystems/Indexer/ExitRollers/GoalSpeedRPM", goalRPM);
            Log.log(
                    "Subsystems/Indexer/ExitRollers/PIDOutputRPM",
                    profiledPIDController.getPositionError());
            Log.log("ROBOT/Subsystems/Indexer/ExitRollers/PIDVolts", pidOutput);
            Log.log("ROBOT/Subsystems/Indexer/ExitRollers/FFVolts", ffOutput);
        }

        // Apply to sim
        leaderflywheelSim.setInputVoltage(voltage);
        leaderflywheelSim.update(0.02);

        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
