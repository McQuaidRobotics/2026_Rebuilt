package igknighters.subsystems.intake.rollers;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.constants.SubsystemConstants;

public class RollersSim extends Rollers {

    private double inputVoltage = 0.0;

    private final FlywheelSim leaderflywheelSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.kShooter.kRollers.MOMENT_OF_INERTIA_KG_M2,
                            SubsystemConstants.kShooter.kRollers.GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    SubsystemConstants.kShooter.kRollers.kI,
                    SubsystemConstants.kShooter.kRollers.kD,
                    new Constraints(
                            SubsystemConstants.kShooter.kRollers.MAX_SPEED_RPM,
                            SubsystemConstants.kShooter.kRollers.MAX_ACCELERATION_RPM));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SubsystemConstants.kShooter.kRollers.kS,
                    SubsystemConstants.kShooter.kRollers.kV,
                    SubsystemConstants.kShooter.kRollers.kA);
    private boolean isPidControlledThisCycle = false;
    private boolean isVoltageControlledThisCycle = false;

    @Override
    public double getSpeedRPM() {
        return leaderflywheelSim.getAngularVelocityRPM();
    }

    @Override
    public void stop() {
        inputVoltage = 0.0;
        isVoltageControlledThisCycle = true;
    }

    @Override
    public void goToSpeedRPM(double speedRPM) {
        DogLog.log("Subsystems/Intake/Rollers/Target Speed RPM", speedRPM);
        profiledPIDController.setGoal(speedRPM);
        isPidControlledThisCycle = true;
    }

    @Override
    public void periodic() {
        double currentRPM = leaderflywheelSim.getAngularVelocityRPM();
        double goalRPM = profiledPIDController.getGoal().position;
        double ffVolts = 0.0;
        double pidVolts = 0.0;
        double voltage = 0.0;

        if (isPidControlledThisCycle) {

            // Feedforward in volts
            ffVolts = feedforward.calculate(goalRPM);

            // PID output is in RPM, convert to volts with a small gain
            // Tune this value (start around 0.001)
            double kRPM_to_volts = 0.001;

            double pidRPM = profiledPIDController.calculate(currentRPM);
            pidVolts = pidRPM * kRPM_to_volts;

            voltage = ffVolts + pidVolts;
        }

        if (isVoltageControlledThisCycle) {
            voltage = inputVoltage;
        }

        // Clamp to real motor limits
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);

        // Logging
        DogLog.log("Subsystems/Shooter/Rollers/SimVoltage", voltage);
        DogLog.log("Subsystems/Shooter/Rollers/SimSpeedRPM", currentRPM);
        DogLog.log("Subsystems/Shooter/Rollers/GoalSpeedRPM", goalRPM);
        DogLog.log("Subsystems/Shooter/Rollers/PIDError", profiledPIDController.getPositionError());
        DogLog.log("Subsystems/Shooter/Rollers/PIDVolts", pidVolts);
        DogLog.log("Subsystems/Shooter/Rollers/FeedforwardVolts", ffVolts);

        // Apply to sim
        leaderflywheelSim.setInputVoltage(voltage);
        leaderflywheelSim.update(0.02);

        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
