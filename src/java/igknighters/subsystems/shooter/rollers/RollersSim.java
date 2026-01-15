package igknighters.subsystems.shooter.rollers;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
                            SubsystemConstants.Shooter.MOMENT_OF_INERTIA_KG_M2,
                            SubsystemConstants.Shooter.GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final ProfiledPIDController profiledPIDController =
            new ProfiledPIDController(
                    .8,
                    SubsystemConstants.Shooter.kI,
                    SubsystemConstants.Shooter.kD,
                    new Constraints(
                            SubsystemConstants.Shooter.MAX_SPEED_RPM,
                            SubsystemConstants.Shooter.MAX_ACCELERATION_RPM));
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
        double voltage = 0.0;
        if (isPidControlledThisCycle) {
            double pidOutput =
                    profiledPIDController.calculate(leaderflywheelSim.getAngularVelocityRPM());
            voltage = (pidOutput / profiledPIDController.getGoal().position) * 12.0;
        }
        if (isVoltageControlledThisCycle) {
            voltage = inputVoltage;
        }
        DogLog.log("Subsystems/Shooter/SimVoltage", voltage);
        DogLog.log("Subsystems/Shooter/SimSpeedRPM", leaderflywheelSim.getAngularVelocityRPM());
        DogLog.log("Subsystems/Shooter/GoalSpeedRPM", profiledPIDController.getGoal().position);
        DogLog.log("Subsystems/Shooter/CommandedVoltage", inputVoltage);
        DogLog.log("Subsystems/Shooter/PIDCONTROLLED", isPidControlledThisCycle);
        DogLog.log("Subsystems/Shooter/VOLTAGECONTROLLED", isVoltageControlledThisCycle);
        leaderflywheelSim.setInputVoltage(voltage);
        leaderflywheelSim.update(0.02);
        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
