package igknighters.subsystems.intake.pivot;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class PivotSim extends Pivot {
    private boolean stop = false;
    private SingleJointedArmSim pivotSim;
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    SubsystemConstants.kShooter.kTurret.kP,
                    SubsystemConstants.kShooter.kTurret.kI,
                    SubsystemConstants.kShooter.kTurret.kD,
                    new TrapezoidProfile.Constraints(
                            SubsystemConstants.kShooter.kTurret.MAX_SPEED_RPM,
                            SubsystemConstants.kShooter.kTurret.MAX_ACCELERATION_RPM));

    public PivotSim() {
        pivotSim =
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

    @Override
    public void stop() {
        stop = true;
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        pivotSim.setState(angleDegrees * Conv.DEGREES_TO_RADIANS, 0.0);
        controller.reset(angleDegrees * Conv.DEGREES_TO_RADIANS);
    }

    private boolean isControlledThisCycle = false;

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Intake/Pivot/Target", angleDegrees);
        controller.setGoal(angleDegrees * Conv.DEGREES_TO_RADIANS);
        isControlledThisCycle = true;
    }

    @Override
    public double getAngleDegrees() {
        return pivotSim.getAngleRads() * Conv.RADIANS_TO_DEGREES;
    }

    @Override
    public void periodic() {
        double input = 0.0;
        if (!stop && isControlledThisCycle) {
            input = controller.calculate(pivotSim.getAngleRads());
            input = input / Math.PI * 12.0; // scale to volts
        }
        pivotSim.setInput(input);
        pivotSim.update(0.020);

        DogLog.log("Subsystems/Intake/Pivot/AngleDegrees", getAngleDegrees());
        DogLog.log(
                "Subsystems/Intake/Pivot/TargetDegrees",
                controller.getGoal().position * Conv.RADIANS_TO_DEGREES);
        DogLog.log("Subsystems/Intake/Pivot/MotorVoltage", input);

        isControlledThisCycle = false;
    }
}
