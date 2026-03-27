package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants.kIntake;
import igknighters.util.log.Log;

public class RollersSim extends Rollers {

    private double inputVoltage = 0.0;

    private final FlywheelSim leaderflywheelSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            kIntake.kRollers.MOMENT_OF_INERTIA_KG_M2,
                            kIntake.kRollers.GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    kIntake.kRollers.kI,
                    kIntake.kRollers.kD,
                    new Constraints(
                            kIntake.kRollers.MAX_SPEED_RPM, kIntake.kRollers.MAX_ACCELERATION_RPM));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    kIntake.kRollers.kS, kIntake.kRollers.kV, kIntake.kRollers.kA);
    private boolean isPidControlledThisCycle = false;
    private boolean isVoltageControlledThisCycle = false;

    @Override
    public AngularVelocity getSpeed() {
        return RPM.of(leaderflywheelSim.getAngularVelocityRPM());
    }

    @Override
    public void stop() {
        inputVoltage = 0.0;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public void goToSpeed(AngularVelocity speed) {
        if (!Robot.consts.intake().kRollers().disableRollersLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Rollers/Target Speed RPM", speed.in(RPM));
        }
        profiledPIDController.setGoal(speed.in(RPM));
        isPidControlledThisCycle = true;
    }

    @Override
    public void periodic() {
        double voltage = 0.0;
        double goalRPM = profiledPIDController.getGoal().position;
        double currentRPM = leaderflywheelSim.getAngularVelocityRPM();

        double pidOutput = 0.0;
        double ffOutput = 0.0;

        if (isPidControlledThisCycle) {

            // Feedforward in volts

            double goalRPS = goalRPM / 60.0;

            ffOutput = kIntake.kRollers.kS + kIntake.kRollers.kV * goalRPS;

            // PID output is in RPM, convert to volts with a small gain
            // Tune this value (start around 0.001)
            double kRPM_to_volts = 0.01;

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
        if (!Robot.consts.intake().kRollers().disableRollersLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Rollers/SimVoltage", voltage);
            Log.log("ROBOT/Subsystems/Intake/Rollers/SimSpeedRPM", currentRPM);
            Log.log("ROBOT/Subsystems/Intake/Rollers/GoalSpeedRPM", goalRPM);
            Log.log(
                    "Subsystems/Intake/Rollers/Pid Error RPM",
                    profiledPIDController.getPositionError());
            Log.log("ROBOT/Subsystems/Intake/Rollers/PIDVolts", pidOutput);
            Log.log("ROBOT/Subsystems/Intake/Rollers/FFVolts", ffOutput);
        }

        // Apply to sim
        leaderflywheelSim.setInputVoltage(voltage);
        leaderflywheelSim.update(0.02);

        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
