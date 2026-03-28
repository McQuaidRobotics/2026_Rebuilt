package igknighters.subsystems.shooter.flywheel;

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
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.util.log.Log;

public class FlywheelSimulator extends Flywheel {

    private double inputVoltage = 0.0;

    private final FlywheelSim leaderflywheelSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.kShooter.kFlywheels.MOMENT_OF_INERTIA_KG_M2,
                            SubsystemConstants.kShooter.kFlywheels.GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    SubsystemConstants.kShooter.kFlywheels.kI,
                    SubsystemConstants.kShooter.kFlywheels.kD,
                    new Constraints(
                            SubsystemConstants.kShooter.kFlywheels.MAX_SPEED_RPM,
                            SubsystemConstants.kShooter.kFlywheels.MAX_ACCELERATION_RPM));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SubsystemConstants.kShooter.kFlywheels.kS,
                    SubsystemConstants.kShooter.kFlywheels.kV,
                    SubsystemConstants.kShooter.kFlywheels.kA);

    private boolean isPidControlledThisCycle = false;

    private boolean isVoltageControlledThisCycle = false;

    @Override
    public void setSpeed(AngularVelocity speed) {

        profiledPIDController.setGoal(speed.in(RPM));

        isPidControlledThisCycle = true;
    }

    @Override
    public void setVoltage(double voltage) {

        inputVoltage = voltage;

        isVoltageControlledThisCycle = true;
    }

    @Override
    public AngularVelocity getSpeed() {

        return RPM.of(leaderflywheelSim.getAngularVelocityRPM());
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

            double goalRPS = goalRPM * Conv.RPM_TO_RPS;

            ffOutput = kShooter.kFlywheels.kS + kShooter.kFlywheels.kV * goalRPS;

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

        if (!Robot.consts.shooter().kFlywheels().disableFlywheelsLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/Rollers/SimVoltage", voltage);

            Log.log("ROBOT/Subsystems/Shooter/Rollers/SimSpeedRPM", currentRPM);

            Log.log("ROBOT/Subsystems/Shooter/Rollers/GoalSpeedRPM", goalRPM);

            Log.log(
                    "Subsystems/Shooter/Rollers/PIDOutputRPM",
                    profiledPIDController.getPositionError());

            Log.log("ROBOT/Subsystems/Shooter/Rollers/PIDVolts", pidOutput);

            Log.log("ROBOT/Subsystems/Shooter/Rollers/FFVolts", ffOutput);
        }

        // Apply to sim

        leaderflywheelSim.setInputVoltage(voltage);

        leaderflywheelSim.update(0.02);

        isPidControlledThisCycle = false;

        isVoltageControlledThisCycle = false;
    }
}
