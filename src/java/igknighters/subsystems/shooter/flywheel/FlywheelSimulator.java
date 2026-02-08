package igknighters.subsystems.shooter.flywheel;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;

public class FlywheelSimulator extends Flywheel {

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
    public void setSpeed(double speedRPM) {

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

            ffOutput = kShooter.kRollers.kS + kShooter.kRollers.kV * goalRPS;

            // PID output is in RPM, convert to volts with a small gain

            double kRPM_to_volts = 0.002;

            double pidRPM = profiledPIDController.calculate(currentRPM);

            pidOutput = pidRPM * kRPM_to_volts;

            voltage = pidOutput + ffOutput;

        } else if (isVoltageControlledThisCycle) {

            voltage = inputVoltage;
        }

        // Clamp to real motor limits

        voltage = MathUtil.clamp(voltage, -12.0, 12.0);

        // Logging

        DogLog.log("Subsystems/Shooter/Rollers/SimVoltage", voltage);

        DogLog.log("Subsystems/Shooter/Rollers/SimSpeedRPM", currentRPM);

        DogLog.log("Subsystems/Shooter/Rollers/GoalSpeedRPM", goalRPM);

        DogLog.log(
                "Subsystems/Shooter/Rollers/PIDOutputRPM",
                profiledPIDController.getPositionError());

        DogLog.log("Subsystems/Shooter/Rollers/PIDVolts", pidOutput);

        DogLog.log("Subsystems/Shooter/Rollers/FFVolts", ffOutput);

        // Apply to sim

        leaderflywheelSim.setInputVoltage(voltage);

        leaderflywheelSim.update(0.02);

        isPidControlledThisCycle = false;

        isVoltageControlledThisCycle = false;
    }
}
