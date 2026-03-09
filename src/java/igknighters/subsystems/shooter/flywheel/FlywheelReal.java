package igknighters.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.util.log.Log;

public class FlywheelReal extends Flywheel {
    private final TalonFX mainShooter =
            new TalonFX(SubsystemConstants.kShooter.kFlywheels.LEADER_MOTOR_ID, kShooter.CANBUS);
    private final TalonFX followerShooter =
            new TalonFX(SubsystemConstants.kShooter.kFlywheels.FOLLOWER_MOTOR_ID, kShooter.CANBUS);

    // private final MotionMagicVelocityVoltage velocityControl = new

    private boolean isBeingControlledActivly = false;
    // MotionMagicVelocityVoltage(0.0);

    private final MotionMagicVelocityVoltage velocityControl;
    // private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
    //         new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // private final DigitalInput beamBreakSensor = new
    // DigitalInput(SubsystemConstants.Shooter.BEAM_BREAK_SENSOR_CHANNEL);

    // private BaseStatusSignal isBeamBreakTripped;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.kShooter.kFlywheels.kP;
        config.Slot0.kI = SubsystemConstants.kShooter.kFlywheels.kI;
        config.Slot0.kD = SubsystemConstants.kShooter.kFlywheels.kD;
        config.Slot0.kS = SubsystemConstants.kShooter.kFlywheels.kS;
        config.Slot0.kV = SubsystemConstants.kShooter.kFlywheels.kV;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.kShooter.kFlywheels.GEAR_RATIO;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotionMagic.MotionMagicJerk =
                SubsystemConstants.kShooter.kFlywheels.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kShooter.kFlywheels.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kShooter.kFlywheels.MAX_SPEED_RPM;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kShooter.kFlywheels.SUPPLY_CURRENT_LIMIT;
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                SubsystemConstants.kShooter.kFlywheels.PEAK_CURRENT_LIMIT;

        return config;
    }

    public FlywheelReal() {

        mainShooter.getConfigurator().apply(getLeaderConfig());
        followerShooter.setControl(
                new Follower(mainShooter.getDeviceID(), MotorAlignmentValue.Opposed));

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

    }

    @Override
    public void setSpeed(AngularVelocity speedRPM) {
        if (!SubsystemConstants.kShooter.kFlywheels.disableFlywheelsLogs) {
            Log.log("ROBOT/Subsystems/Shooter/Flywheels/setSpeed", speedRPM);
        }
        isBeingControlledActivly = true;
        mainShooter.setControl(velocityControl.withVelocity(speedRPM.in(RotationsPerSecond)));
    }

    @Override
    public void setVoltage(double voltage) {
        isBeingControlledActivly = true;
        mainShooter.setControl(dutyCycleControl.withOutput(voltage / 12.0));
    }

    @Override
    public AngularVelocity getSpeed() {
        return mainShooter.getVelocity().getValue();
    }

    @Override
    public void periodic() {
        if (!SubsystemConstants.kShooter.kFlywheels.disableFlywheelsLogs) {
            Log.log("ROBOT/Subsystems/Shooter/Flywheels/being controlled", isBeingControlledActivly);
            Log.logMotor("ROBOT/Subsystems/Shooter/Flywheels/Motor", mainShooter);
        }

        isBeingControlledActivly = false;
    }
}
