package igknighters.subsystems.shooter.turret;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class TurretSim extends Turret {

    private SingleJointedArmSim turretSim;
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    SubsystemConstants.kShooter.kTurret.kP,
                    SubsystemConstants.kShooter.kTurret.kI,
                    SubsystemConstants.kShooter.kTurret.kD,
                    new TrapezoidProfile.Constraints(
                            SubsystemConstants.kShooter.kTurret.MAX_SPEED_RPM,
                            SubsystemConstants.kShooter.kTurret.MAX_ACCELERATION_RPM));

    public TurretSim() {
        turretSim =
                new SingleJointedArmSim(
                        LinearSystemId.createSingleJointedArmSystem(
                                DCMotor.getKrakenX60(1),
                                SubsystemConstants.kShooter.kFlywheels.MOMENT_OF_INERTIA_KG_M2,
                                SubsystemConstants.kShooter.kTurret.GEAR_RATIO),
                        DCMotor.getKrakenX60(1),
                        SubsystemConstants.kShooter.kTurret.GEAR_RATIO,
                        .2, // this is not a number that i know it is mainly for gravity sim which
                        // we dont need
                        SubsystemConstants.kShooter.kTurret.MIN_ANGLE_DEGREES / 360 * 2 * Math.PI,
                        SubsystemConstants.kShooter.kTurret.MAX_ANGLE_DEGREES / 360 * 2 * Math.PI,
                        false,
                        0.0);
    }

    private boolean isControlledThisCycle = false;

    @Override
    public void setAngleDegrees(double angleDegrees) {
        super.degrees = angleDegrees;
        turretSim.setState(angleDegrees * Conv.DEGREES_TO_RADIANS, 0.0);
        controller.reset(angleDegrees * Conv.DEGREES_TO_RADIANS);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        controller.setGoal(angleDegrees * Conv.DEGREES_TO_RADIANS);
        isControlledThisCycle = true;
    }

    @Override
    public double getAngleDegrees() {
        return turretSim.getAngleRads() * Conv.RADIANS_TO_ROTATIONS * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void periodic() {
        double input = 0.0;
        if (isControlledThisCycle) {
            input = controller.calculate(turretSim.getAngleRads());
            input = input / Math.PI * 12.0; // scale to volts
        }

        turretSim.setInput(input);
        turretSim.update(0.020);

        DogLog.log("Subsystems/Shooter/Turret/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Turret/TargetDegrees", super.targetDegrees);
        DogLog.log("Subsystems/Shooter/Turret/MotorVoltage", input);

        isControlledThisCycle = false;
    }
}
