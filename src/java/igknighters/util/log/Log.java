package igknighters.util.log;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.constants.Conv;
import org.littletonrobotics.junction.Logger;

public class Log {

    // ==========================================
    // MOTORS
    // ==========================================

    public static void logMotor(String path, TalonFX motor) {
        Logger.recordOutput(path + "/OutputCurrent", motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput(
                path + "/Velocity RPM", motor.getVelocity().getValueAsDouble() * Conv.RPS_TO_RPM);
        Logger.recordOutput(
                path + "/Position DEG",
                motor.getPosition().refresh().getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES);
    }

    public static void logMotorMinimal(String path, TalonFX motor) {
        Logger.recordOutput(
                path + "/Position DEG",
                motor.getPosition().getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES);
        Logger.recordOutput(
                path + "/Velocity RPM", motor.getVelocity().getValueAsDouble() * Conv.RPS_TO_RPM);
    }

    // ==========================================
    // PRIMITIVES & STRINGS
    // ==========================================

    public static void log(String path, double value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, double[][] value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, ChassisSpeeds[] value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, boolean value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, int value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, long value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, String value) {
        Logger.recordOutput(path, value);
    }

    // ==========================================
    // PRIMITIVE ARRAYS
    // ==========================================

    public static void log(String path, double[] value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, boolean[] value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, long[] value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, String[] value) {
        Logger.recordOutput(path, value);
    }

    // ==========================================
    // UNITS
    // ==========================================

    public static void log(String path, AngularVelocity value) {
        Logger.recordOutput(path, value.in(RPM));
    }

    // ==========================================
    // GEOMETRY
    // ==========================================

    public static void log(String path, Translation2d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Translation3d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Rotation2d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Rotation3d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Pose2d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Pose3d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Transform2d value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Transform3d value) {
        Logger.recordOutput(path, value);
    }

    // ==========================================
    // GEOMETRY ARRAYS (Using Varargs for flexibility)
    // ==========================================

    public static void log(String path, Pose2d... value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Pose3d... value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Translation2d... value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, Translation3d... value) {
        Logger.recordOutput(path, value);
    }

    // ==========================================
    // KINEMATICS & SWERVE
    // ==========================================

    public static void log(String path, ChassisSpeeds value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, SwerveModuleState... value) {
        Logger.recordOutput(path, value);
    }

    public static void log(String path, SwerveModulePosition... value) {
        Logger.recordOutput(path, value);
    }

    // ==========================================
    // OTHER WPILIB TYPES
    // ==========================================

    public static void log(String path, Trajectory value) {
        Logger.recordOutput(path, value);
    }

    // AdvantageKit doesn't natively log the Color object, so we convert it to a Hex String
    public static void log(String path, Color value) {
        Logger.recordOutput(path, value.toHexString());
    }

    public static void log(String path, Color8Bit value) {
        Logger.recordOutput(path, value.toHexString());
    }
}
