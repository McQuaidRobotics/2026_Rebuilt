package igknighters.subsystems.intake.rollers;

import dev.doglog.DogLog;
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
        double voltage = 0.0;
        if (isPidControlledThisCycle) {
            double pidOutput =
                    profiledPIDController.calculate(leaderflywheelSim.getAngularVelocityRPM());
            double feedforwardOutput =
                    (feedforward.calculate(profiledPIDController.getGoal().position)
                                    / profiledPIDController.getGoal().position)
                            * 12.0;

            DogLog.log("Subsystems/Shooter/Rollers/PIDOutput", pidOutput);
            voltage =
                    (pidOutput / profiledPIDController.getGoal().position) * 12.0
                            + feedforwardOutput;
        }
        if (isVoltageControlledThisCycle) {
            voltage = inputVoltage;
        }
        DogLog.log("Subsystems/Shooter/Rollers/SimVoltage", voltage);
        DogLog.log(
                "Subsystems/Shooter/Rollers/SimSpeedRPM",
                leaderflywheelSim.getAngularVelocityRPM());
        DogLog.log(
                "Subsystems/Shooter/Rollers/GoalSpeedRPM",
                profiledPIDController.getGoal().position);
        DogLog.log("Subsystems/Shooter/Rollers/CommandedVoltage", inputVoltage);
        DogLog.log("Subsystems/Shooter/Rollers/PIDCONTROLLED", isPidControlledThisCycle);
        DogLog.log("Subsystems/Shooter/Rollers/VOLTAGECONTROLLED", isVoltageControlledThisCycle);
        leaderflywheelSim.setInputVoltage(voltage);
        leaderflywheelSim.update(0.02);
        isPidControlledThisCycle = false;
        isVoltageControlledThisCycle = false;
    }
}
