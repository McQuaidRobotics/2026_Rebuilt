package igknighters.subsystems.shooter.hood.flap;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class FlapSim extends Flap {
    private SingleJointedArmSim flapSim =
            new SingleJointedArmSim(
                    LinearSystemId.createSingleJointedArmSystem(
                            DCMotor.getKrakenX60(1),
                            SubsystemConstants.Hood.JKG_M2,
                            SubsystemConstants.Hood.GEAR_RATIO),
                    DCMotor.getKrakenX60(1),
                    SubsystemConstants.Hood.GEAR_RATIO,
                    SubsystemConstants.Hood.LENGTH_METERS,
                    SubsystemConstants.Hood.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS,
                    SubsystemConstants.Hood.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS,
                    true,
                    SubsystemConstants.Hood.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS);
    private ProfiledPIDController pidController =
            new ProfiledPIDController(
                    SubsystemConstants.Hood.kP,
                    SubsystemConstants.Hood.kI,
                    SubsystemConstants.Hood.kD,
                    new TrapezoidProfile.Constraints(
                            SubsystemConstants.Hood.MAX_SPEED_RPM * 360.0,
                            SubsystemConstants.Hood.MAX_ACCELERATION_RPM * 360.0)); // in degrees per minute

    @Override
    public void periodic() {
        double input = pidController.calculate(getAngleDegrees());
        flapSim.setInput(input);
        flapSim.update(0.02); // Update the simulation with a 20ms timestep

        DogLog.log("Subsystems/Shooter/Hood/Flap/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Hood/Flap/TargetDegrees", super.targetDegrees);

    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        flapSim.setState(angleDegrees * Conv.DEGREES_TO_RADIANS, 0.0);
    }
    // pid controllers gains are in degrees
    @Override
    public void goToAngleDegrees(double angleDegrees) {
        pidController.setGoal(angleDegrees);

    }

    @Override
    public double getAngleDegrees() {
        return flapSim.getAngleRads() * Conv.RADIANS_TO_DEGREES;
    }
}
