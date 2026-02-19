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

    private SingleJointedArmSim pivotSim;
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    SubsystemConstants.kIntake.kPivot.kP,
                    SubsystemConstants.kIntake.kPivot.kI,
                    SubsystemConstants.kIntake.kPivot.kD,
                    new TrapezoidProfile.Constraints(
                            SubsystemConstants.kIntake.kPivot.MAX_SPEED_METERS_PER_SECOND,
                            SubsystemConstants.kIntake
                                    .kPivot
                                    .MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    public PivotSim() {
        pivotSim =
                new SingleJointedArmSim(
                        LinearSystemId.createSingleJointedArmSystem(
                                DCMotor.getKrakenX60(1),
                                SubsystemConstants.kIntake.kPivot.JKG_M2,
                                SubsystemConstants.kIntake.kPivot.GEAR_RATIO),
                        DCMotor.getKrakenX60(1),
                        SubsystemConstants.kIntake.kPivot.GEAR_RATIO,
                        SubsystemConstants.kIntake.kPivot.LENGTH_METERS,
                        SubsystemConstants.kIntake.kPivot.MIN_ANGLE_DEGREES
                                * Conv.DEGREES_TO_RADIANS,
                        SubsystemConstants.kIntake.kPivot.MAX_ANGLE_DEGREES
                                * Conv.DEGREES_TO_RADIANS,
                        false,
                        0.0);
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        super.degrees = angleDegrees;
        pivotSim.setState(angleDegrees * Conv.DEGREES_TO_RADIANS, 0.0);
        controller.reset(angleDegrees * Conv.DEGREES_TO_RADIANS);
    }

    private boolean isControlledThisCycle = false;

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        super.targetDegrees = angleDegrees;
        controller.setGoal(angleDegrees * Conv.DEGREES_TO_RADIANS);
        isControlledThisCycle = true;
    }

    @Override
    public double getAngleDegrees() {
        return pivotSim.getAngleRads() * Conv.RADIANS_TO_DEGREES;
    }

    @Override
    public void stop() {
        // Not implemented in TurretSim, so I'll leave it blank for now.
        // I'll set the goal to the current angle to stop the motor.
        controller.setGoal(pivotSim.getAngleRads());
    }

    @Override
    public void periodic() {
        double input = controller.calculate(pivotSim.getAngleRads());

        // This scaling is likely incorrect, but I'm following the TurretSim example.
        input = input / Math.PI * 12.0; // scale to volts

        pivotSim.setInput(input);
        pivotSim.update(0.020);

        DogLog.log("Subsystems/Intake/Pivot/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Intake/Pivot/TargetDegrees", super.targetDegrees);
        DogLog.log("Subsystems/Intake/Pivot/MotorVoltage", input);
    }
}
