package igknighters.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.util.log.Log;

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
    public void setAngle(Angle angle) {
        super.degrees = angle.in(Degrees);
        pivotSim.setState(angle.in(Radians), 0.0);
        controller.reset(angle.in(Radians));
    }

    private boolean isControlledThisCycle = false;

    @Override
    public void goToAngle(Angle angle) {
        super.targetDegrees = angle.in(Degrees);
        controller.setGoal(angle.in(Radians));
        isControlledThisCycle = true;
    }

    @Override
    public Angle getAngle() {
        return Radians.of(pivotSim.getAngleRads());
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

        
        if (!SubsystemConstants.kIntake.kPivot.disablePivotLogs)
        {
        Log.log("Subsystems/Intake/Pivot/AngleDegrees", getAngle().in(Degrees));
        Log.log("Subsystems/Intake/Pivot/TargetDegrees", super.targetDegrees);
        Log.log("Subsystems/Intake/Pivot/MotorVoltage", input);
        }
    }
}
