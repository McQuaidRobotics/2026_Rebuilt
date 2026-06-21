package igknighters.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class GeminiRobotConsts extends RobotConsts {

    public static final int firstMotorID = 15;
    public static final boolean disableAllLogs = false;
    public static final CANBus superStructure = new CANBus("SuperStructureBus");
    public static final CANBus drive = new CANBus("DriveBus");

    @Override
    public SWERVE_CONSTS swerve() {
        return new GeminiSwerveConsts();
    }

    public static class GeminiSwerveConsts implements SWERVE_CONSTS {
        static SwerveConsts swerveConsts = new SwerveConsts();

        static CommonSwerveConsts commonSwerveConsts = swerveConsts.getSwerveConsts();

        @Override
        public CommonSwerveConsts getCommonSwerveConsts() {
            return commonSwerveConsts;
        }
    }

    @Override
    public CANBus getSuperStructureBus() {
        return superStructure;
    }

    @Override
    public boolean disableAllLogs() {
        return disableAllLogs;
    }

    @Override
    public kShooterConsts shooter() {
        return new GeminiShooterConsts();
    }

    @Override
    public kIndexerConsts indexer() {
        return new GeminiIndexerConsts();
    }

    @Override
    public kIntakeConsts intake() {
        return new GeminiIntakeConsts();
    }

    @Override
    public kLimelightVisionConsts limelightVision() {
        return new GeminiLimelightVisionConsts();
    }

    public static class GeminiIntakeConsts implements kIntakeConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kPivotConsts kPivot() {
            return new GeminiPivotConsts();
        }

        @Override
        public kRollersConsts kRollers() {
            return new GeminiRollersConsts();
        }
    }

    public static class GeminiPivotConsts implements kPivotConsts {
        @Override
        public int MOTOR_ID() {
            return 15;
        }

        @Override
        public SmartMotorControllerConfig getConfig(CANcoder caNcoder, Subsystem subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)

                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(45.0, 0.0, 0.0, ClosedLoopControllerSlot.SLOT_0)
                    .withSimClosedLoopController(
                            .2,
                            0.05,
                            0.0,
                            ClosedLoopControllerSlot
                                    .SLOT_0) // these values are just for simulation and should
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(20), RotationsPerSecondPerSecond.of(40))
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0.0, 0, 0.0, 0.0))
                    .withSimFeedforward(new ArmFeedforward(0.0, 0, 0.0, 0.0))
                    // Telemetry name and verbosity level
                    .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withSoftLimits(
                            Degrees.of(MIN_ANGLE_DEGREES()), Degrees.of(MAX_ANGLE_DEGREES()))
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.BRAKE)
                    .withExternalEncoder(caNcoder)
                    .withExternalEncoderGearing(1)
                    .withExternalEncoderZeroOffset(Rotations.of(0.31982421875))
                    .withExternalEncoderInverted(true)
                    .withUseExternalFeedbackEncoder(true)
                    .withStatorCurrentLimit(Amps.of(20))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25));
        }

        @Override
        public double STOWED_ANGLE_DEGREES() {
            return 16.0;
        }

        @Override
        public double PARTIAL_STOW() {
            return 66.0;
        }

        @Override
        public double JORK_ANGLE() {
            return 40.0;
        }

        @Override
        public int CANCODER_ID() {
            return 21;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 66.0;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return 0.0;
        }

        @Override
        public double ENCODER_OFFSET() {
            return 0.31982421875;
        }

        @Override
        public boolean disablePivotLogs() {
            return false || disableAllLogs;
        }
    }

    public static class GeminiRollersConsts implements kRollersConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 16;
        }

        @Override
        public SmartMotorControllerConfig getConfig(Subsystem subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(0.3, 0.0, 0.0)
                    .withFollowers(
                            Pair.of(
                                    new TalonFX(
                                            Robot.consts.intake().kRollers().FOLLOWER_MOTOR_ID()),
                                    false))
                    .withSimClosedLoopController(0.1, 0.0, 0.0)
                    // Feedforward Constants
                    .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
                    // Telemetry name and verbosity level
                    .withTelemetry("Intake Rollers", TelemetryVerbosity.HIGH)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(25));
        }

        @Override
        public int FOLLOWER_MOTOR_ID() {
            return 17;
        }

        @Override
        public boolean disableRollersLogs() {
            return true || disableAllLogs;
        }
    }

    public static class GeminiShooterConsts implements kShooterConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kFlywheelsConsts kFlywheels() {
            return new GeminiFlywheelsConsts();
        }

        @Override
        public kTurretConsts kTurret() {
            return new GeminiTurretConsts();
        }

        @Override
        public kHoodConsts kHood() {
            return new GeminiHoodConsts();
        }
    }

    @Override
    public kLerpConsts lerp() {
        return new GeminiLerpConsts();
    }

    public static class GeminiLerpConsts implements kLerpConsts {
        @Override
        public kRPMConsts kRPM() {
            return new GeminiRPMConsts();
        }

        @Override
        public kHoodAngleConsts kHoodAngle() {
            return new GeminiHoodLerpAngleConsts();
        }

        @Override
        public kRadialSOTMConsts kRadialSOTM() {
            return new GeminiRadialSOTMConsts();
        }

        @Override
        public kTangentialSOTMConsts kTangentialSOTM() {
            return new GeminiTangentialSOTMConsts();
        }

        @Override
        public kTOFConsts kTimeOfFlight() {
            return new GeminiTOFConsts();
        }
    }

    public static class GeminiTangentialSOTMConsts implements kTangentialSOTMConsts {
        static LerpTable TANGENTIAL =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, .7),
                            new LerpTableEntry(2.0, .8),
                            new LerpTableEntry(3, .9),
                            new LerpTableEntry(4.5, 1.0),
                            new LerpTableEntry(5, 1.1)
                        });

        @Override
        public LerpTable table() {
            return TANGENTIAL;
        }
    }

    public static class GeminiRadialSOTMConsts implements kRadialSOTMConsts {
        static LerpTable RADIAL_TOWARDS =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, .6),
                            new LerpTableEntry(2.0, .7),
                            new LerpTableEntry(3, .8),
                            new LerpTableEntry(4.5, .9),
                            new LerpTableEntry(5, 1.0)
                        });

        static LerpTable RADIAL_AWAY =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, .6),
                            new LerpTableEntry(2.0, .7),
                            new LerpTableEntry(3, .8),
                            new LerpTableEntry(4.5, .9),
                            new LerpTableEntry(5, 1.0)
                        });

        @Override
        public LerpTable away() {
            return RADIAL_AWAY;
        }

        @Override
        public LerpTable towards() {
            return RADIAL_TOWARDS;
        }
    }

    public static class GeminiHoodLerpAngleConsts implements kHoodAngleConsts {
        static LerpTable HOOD_LERP =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(
                                    1.5,
                                    kHood.MIN_ANGLE_DEGREES), // this is a shortfall but i dont know
                            // how to acces other consts within
                            // this file
                            new LerpTableEntry(2.5, 25.0),
                            new LerpTableEntry(3.5, 30.0),
                            new LerpTableEntry(4.5, 33.0),
                            new LerpTableEntry(5.5, 35.0),
                            new LerpTableEntry(6.0, 38.0),
                            new LerpTableEntry(10.0, 45)
                        });

        @Override
        public LerpTable table() {
            return HOOD_LERP;
        }
    }

    public static class GeminiRPMConsts implements kRPMConsts {
        static LerpTable RPM_LERP =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.5, 2700),
                            new LerpTableEntry(2.0, 2800),
                            new LerpTableEntry(2.5, 2900),
                            new LerpTableEntry(3.5, 3200),
                            new LerpTableEntry(4.0, 3300),
                            new LerpTableEntry(4.5, 3400),
                            new LerpTableEntry(5.2, 3680),
                            new LerpTableEntry(5.5, 3780),
                            new LerpTableEntry(6.0, 3800),
                            new LerpTableEntry(8.0, 4000),
                            new LerpTableEntry(10.0, 4200),
                            new LerpTableEntry(20, 5500)
                        });

        @Override
        public LerpTable table() {
            return RPM_LERP;
        }
    }

    public static class GeminiTOFConsts implements kTOFConsts {
        static LerpTable TIME_OF_FLIGHT_LERP =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1, 1.1),
                            new LerpTableEntry(2.5, 1.1),
                            new LerpTableEntry(3.5, 1.1),
                            new LerpTableEntry(4, 1.1),
                            new LerpTableEntry(4.5, 1.1),
                            new LerpTableEntry(5.5, 1.15),
                            new LerpTableEntry(6, 1)
                        });

        @Override
        public LerpTable table() {
            return TIME_OF_FLIGHT_LERP;
        }
    }

    public static class GeminiFlywheelsConsts implements kFlywheelsConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 21;
        }

        @Override
        public SmartMotorControllerConfig getConfig(Subsystem subsystem, TalonFX follower) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(0.3, 0.1, 0.0, ClosedLoopControllerSlot.SLOT_0)
                    .withTrapezoidalProfile(
                            RPM.of(5800), RotationsPerSecondPerSecond.of(5800.0 / 60.0 * 1.5))
                    .withFollowers(Pair.of(follower, true))
                    .withSimClosedLoopController(0.3, 0.1, 0.0, ClosedLoopControllerSlot.SLOT_0)
                    // Feedforward Constants
                    .withFeedforward(new SimpleMotorFeedforward(0.17, 0.1, 0.02))
                    .withSimFeedforward(new SimpleMotorFeedforward(0.17, 0.1, 0.02))
                    // Telemetry name and verbosity level
                    .withTelemetry("Flywheels", TelemetryVerbosity.HIGH)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40));
        }

        @Override
        public int FOLLOWER_MOTOR_ID() {
            return 22;
        }

        @Override
        public double WHEEL_RADIUS_METERS() {
            return 0.0508;
        }

        @Override
        public double ShooterHeightMeters() {
            return 0.5;
        }

        @Override
        public boolean disableFlywheelsLogs() {
            return false;
        }
    }

    public static class GeminiTurretConsts implements kTurretConsts {
        @Override
        public int MOTOR_ID() {
            return 18;
        }

        @Override
        public SmartMotorControllerConfig getConfig(Subsystem subsystem, CANcoder caNcoder) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)

                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(75.0, 0.15, 0.0, ClosedLoopControllerSlot.SLOT_0)
                    .withSimClosedLoopController(4, 0.05, 0.0, ClosedLoopControllerSlot.SLOT_0)
                    .withTrapezoidalProfile(
                            RPM.of(200), RotationsPerSecondPerSecond.of(300.0 / 60.0))
                    .withSoftLimits(
                            Degrees.of(MIN_ANGLE_DEGREES()), Degrees.of(MAX_ANGLE_DEGREES()))
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0.0, 0, 0.0, 0.0))
                    .withSimFeedforward(new ArmFeedforward(0.0, 0, 0.0, 0.0))
                    // Telemetry name and verbosity level
                    .withTelemetry("Turret Motor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(16.2)))
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.BRAKE)
                    .withExternalEncoder(caNcoder)
                    .withExternalEncoderGearing(1)
                    .withExternalEncoderZeroOffset(Rotations.of(-0.250732421875))
                    .withExternalEncoderInverted(true)
                    .withUseExternalFeedbackEncoder(true)
                    .withStatorCurrentLimit(Amps.of(20))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25));
        }

        @Override
        public double TURRET_ROBOT_DISTANCE_FROM_CENTERS() {
            return 7.0710678118655;
        }

        @Override
        public int CANCODER_ID() {
            return 19;
        }

        @Override
        public double GEAR_RATIO() {
            return 16.2;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 90.0;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return -270.0;
        }

        @Override
        public boolean disableTurretLogs() {
            return false;
        }
    }

    public static class GeminiHoodConsts implements kHoodConsts {
        @Override
        public int MOTOR_ID() {
            return 20;
        }

        @Override
        public SmartMotorControllerConfig getConfig(Subsystem subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)

                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(4.0, 0.0, 0.0, ClosedLoopControllerSlot.SLOT_0)
                    .withSimClosedLoopController(6, 0.05, 0.0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0.27, 0, 0.0, 0.0))
                    .withSimFeedforward(new ArmFeedforward(0.27, 0, 0.0, 0.0))
                    // Telemetry name and verbosity level
                    .withTelemetry("HOOD Motor", TelemetryVerbosity.HIGH)
                    .withSoftLimits(
                            Degrees.of(MIN_ANGLE_DEGREES()), Degrees.of(MAX_ANGLE_DEGREES()))
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(
                                            360.0 / MOTOR_ROTS_TO_HOOD_DEGREES()))) // 24:1
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(20))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25));
        }

        @Override
        public double MOTOR_ROTS_TO_HOOD_ROTS() {
            // 1 rotation of motor = 15 degrees of hood movement
            // 15 degrees is 15/360 rotations
            // 1 rotation of motor is 0.04 rotations
            return 15.0 / 360.0;
        }

        @Override
        public double MOTOR_ROTS_TO_HOOD_DEGREES() {
            return 15.0;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 52.855225;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return 18.6;
        }

        @Override
        public double LENGTH_METERS() {
            return 0.25;
        }

        @Override
        public int REVERSE_LIMIT_SWITCH_ID() {
            return 9;
        }

        @Override
        public boolean disableHoodLogs() {
            return true;
        }
    }

    public static class GeminiIndexerConsts implements kIndexerConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kSpindexerConsts kSpindexer() {
            return new GeminiSpindexerConsts();
        }

        @Override
        public kExitRollersConsts kExitRollers() {
            return new GeminiExitRollersConsts();
        }
    }

    public static class GeminiSpindexerConsts implements kSpindexerConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 13;
        }

        @Override
        public SmartMotorControllerConfig getConfig(Subsystem subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(50, 0, 0, ClosedLoopControllerSlot.SLOT_0)
                    .withSimClosedLoopController(50, 0, 0, ClosedLoopControllerSlot.SLOT_0)
                    // Feedforward Constants
                    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    // Telemetry name and verbosity level
                    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    // You could also use .withGearing(12) which does the same thing.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40));
        }

        @Override
        public double WHEEL_RADIUS_METERS() {
            return 0.0508;
        }

        @Override
        public double GEAR_RATIO() {
            return 4.0;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.02;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 5000.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 1500.0;
        }

        @Override
        public double MOTION_MAGIC_JERK() {
            return 600.0;
        }

        @Override
        public int BEAM_BREAK_SENSOR_CHANNEL() {
            return 0;
        }

        @Override
        public int FORWARD_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public int REVERSE_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public int STATOR_CURRENT_LIMIT() {
            return 35;
        }

        @Override
        public int SUPPLY_CURRENT_LIMIT() {
            return 25;
        }

        @Override
        public int PEAK_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public double kP() {
            return 0.5;
        }

        @Override
        public double kI() {
            return 0;
        }

        @Override
        public double kD() {
            return 0.001;
        }

        @Override
        public double kS() {
            return 0.178;
        }

        @Override
        public double kV() {
            return 0.5;
        }

        @Override
        public double kA() {
            return 0;
        }

        @Override
        public boolean disableSpindexerLogs() {
            return true;
        }
    }

    public static class GeminiExitRollersConsts implements kExitRollersConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 14;
        }

        @Override
        public SmartMotorControllerConfig getConfig(Subsystem subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(50, 0, 0, ClosedLoopControllerSlot.SLOT_0)
                    .withSimClosedLoopController(50, 0, 0, ClosedLoopControllerSlot.SLOT_0)
                    // Feedforward Constants
                    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    // Telemetry name and verbosity level
                    .withTelemetry("EXIT ROLLERS", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    // You could also use .withGearing(12) which does the same thing.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40));
        }

        @Override
        public double GEAR_RATIO() {
            return 5.0;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.02;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 5000.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 1500.0;
        }

        @Override
        public double MOTION_MAGIC_JERK() {
            return 5000.0;
        }

        @Override
        public int BEAM_BREAK_SENSOR_CHANNEL() {
            return 0;
        }

        @Override
        public double kP() {
            return 0.3;
        }

        @Override
        public double kI() {
            return 0.1;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.3;
        }

        @Override
        public double kV() {
            return 0.1;
        }

        @Override
        public double kA() {
            return 0.02;
        }

        @Override
        public double PEAK_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public double SUPPLY_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public boolean disableExitRollersLogs() {
            return true;
        }
    }

    public static class GeminiLimelightVisionConsts implements kLimelightVisionConsts {
        @Override
        public String turretCam() {
            return "limelight";
        }

        @Override
        public String intakeCam() {
            return "limelight-intake";
        }

        @Override
        public String backCam() {
            return "limelight-back";
        }

        @Override
        public String rightCam() {
            return "limelight-left";
        }

        @Override
        public boolean disableVisionLogs() {
            return true;
        }
    }
}
