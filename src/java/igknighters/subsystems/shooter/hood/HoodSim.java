package igknighters.subsystems.shooter.hood;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.SubsystemConstants;

public class HoodSim extends Hood {
    private final SingleJointedArmSim flapSim =
            new SingleJointedArmSim(
                    LinearSystemId.createSingleJointedArmSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.kShooter.kHood.JKG_M2,
                            SubsystemConstants.kShooter.kHood.MOTOR_ROTS_TO_HOOD_DEGREES),
                    DCMotor.getKrakenX60(1),
                    SubsystemConstants.kShooter.kHood.MOTOR_ROTS_TO_HOOD_DEGREES,
                    SubsystemConstants.kShooter.kHood.LENGTH_METERS,
                    Math.toRadians(SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES),
                    Math.toRadians(SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES),
                    false, // Assuming horizontal/no gravity effect for now
                    Math.toRadians(SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES));

    private final ProfiledPIDController pidController =
            new ProfiledPIDController(
                    SubsystemConstants.kShooter.kHood.kP,
                    SubsystemConstants.kShooter.kHood.kI,
                    SubsystemConstants.kShooter.kHood.kD,
                    new TrapezoidProfile.Constraints(
                            SubsystemConstants.kShooter.kHood.MAX_SPEED_R_P_S,
                            SubsystemConstants.kShooter.kHood.MAX_ACCEL_R_P_S_S));

    private boolean isControlledThisCycle = false;

    @Override
    public void periodic() {
        double voltageInput = 0.0;

        if (isControlledThisCycle) {
            // PID calculation is performed in DEGREES
            voltageInput = pidController.calculate(getAngleDegrees());

            // Standard motor voltage clamp
            voltageInput = MathUtil.clamp(voltageInput, -12.0, 12.0);
        }

        flapSim.setInput(voltageInput);
        flapSim.update(0.02); // Standard 20ms simulation step

        DogLog.log("Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
        DogLog.log("Subsystems/Shooter/Hood/Voltage", voltageInput);

        isControlledThisCycle = false;
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        // State must be stored in RADIANS for the WPILib Sim
        flapSim.setState(Math.toRadians(angleDegrees), 0.0);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        super.targetDegrees = angleDegrees;
        // Goal is DEGREES to match your kP (Volts per Degree)
        pidController.setGoal(angleDegrees);
        isControlledThisCycle = true;
    }

    @Override
    public double getAngleDegrees() {
        // Convert RADIANS from sim back to DEGREES for your robot logic
        return Math.toDegrees(flapSim.getAngleRads());
    }
}
