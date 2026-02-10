package igknighters.subsystems.shooter.hood;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class HoodSim extends Hood {
    private SingleJointedArmSim flapSim =
            new SingleJointedArmSim(
                    LinearSystemId.createSingleJointedArmSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.kShooter.kHood.JKG_M2,
                            SubsystemConstants.kShooter.kHood.GEAR_RATIO),
                    DCMotor.getKrakenX60(1),
                    SubsystemConstants.kShooter.kHood.GEAR_RATIO,
                    SubsystemConstants.kShooter.kHood.LENGTH_METERS,
                    SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS,
                    SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS,
                    true,
                    SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS);
    private ProfiledPIDController pidController =
            new ProfiledPIDController(
                    SubsystemConstants.kShooter.kHood.kP,
                    SubsystemConstants.kShooter.kHood.kI,
                    SubsystemConstants.kShooter.kHood.kD,
                    new TrapezoidProfile.Constraints(
                            SubsystemConstants.kShooter.kHood.MAX_SPEED_RPM * 360.0,
                            SubsystemConstants.kShooter.kHood.MAX_ACCELERATION_RPM
                                    * 360.0)); // in degrees per minute

    private boolean isControlledThisCycle = false;

    @Override
    public void periodic() {
        double input = 0.0;
        if (isControlledThisCycle) {
            input = pidController.calculate(getAngleDegrees());
            input =
                    input
                            / (SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES
                                    - SubsystemConstants.kShooter
                                            .kHood
                                            .MIN_ANGLE_DEGREES); // normalize to -1 to 1
            input = input * 12.0; // scale to voltage
        }
        flapSim.setInput(input);
        flapSim.update(0.02); // Update the simulation with a 20ms timestep

        DogLog.log("Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
        DogLog.log("Subsystems/Shooter/Hood/Voltage", input);

        isControlledThisCycle = false;
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        flapSim.setState(angleDegrees * Conv.DEGREES_TO_RADIANS, 0.0);
    }

    // pid controllers gains are in degrees
    @Override
    public void goToAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/Hood/GoalDegrees", angleDegrees);
        pidController.setGoal(angleDegrees);
        isControlledThisCycle = true;
    }

    @Override
    public double getAngleDegrees() {
        return flapSim.getAngleRads() * Conv.RADIANS_TO_DEGREES;
    }
}
