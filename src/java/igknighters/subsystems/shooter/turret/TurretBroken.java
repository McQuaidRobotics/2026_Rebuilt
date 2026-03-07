package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;

public class TurretBroken extends Turret {

    @Override
    public void setAngle(Angle angle) {
        ;
    }

    public boolean isLegalPosition(double angleDegrees) {
        return angleDegrees >= SubsystemConstants.kShooter.kTurret.MIN_ANGLE_DEGREES
                && angleDegrees <= SubsystemConstants.kShooter.kTurret.MAX_ANGLE_DEGREES;
    }

    public boolean isLegalPositionWrapped(double angleDegrees) {
        double wrappedAngleDegrees = wrapAngleDegrees(angleDegrees);
        return isLegalPosition(wrappedAngleDegrees);
    }

    @Override
    public void goToAngleDegrees(Angle angleDegrees) {
        super.targetDegrees = angleDegrees.in(Degrees);
        double wrappedAngleDegrees = wrapAngleDegrees(angleDegrees.in(Degrees));
        if (!isLegalPositionWrapped(angleDegrees.in(Degrees))) {
            DriverStation.reportError(
                    "Turret angle out of bounds: "
                            + wrappedAngleDegrees
                            + " degrees. Commanded: "
                            + angleDegrees,
                    false);
            return;
        }
        motor.setControl(
                positionControl.withPosition(wrappedAngleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    public double getAngleDegrees() {
        return turretAngle.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(turretAngle);
        if (!SubsystemConstants.kShooter.kTurret.disableTurretLogs) {
            Log.logMotor("Subsystems/Shooter/Turret/Motor", motor);
            Log.log("ROBOT/Subsystems/Shooter/Turret/Target Degrees", super.targetDegrees);
        }

        super.degrees = turretAngle.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }
}
