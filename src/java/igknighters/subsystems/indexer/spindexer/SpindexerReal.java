package igknighters.subsystems.indexer.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import igknighters.constants.SubsystemConstants;

public class SpindexerReal extends Spindexer {
    private final TalonFX mainShooter = new TalonFX(SubsystemConstants.kIndexer.kSpindexer.LEADER_MOTOR_ID);

    // private final MotionMagicVelocityVoltage velocityControl = new
    // MotionMagicVelocityVoltage(0.0);

    private final MotionMagicVelocityVoltage velocityControl;
    // private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
    //         new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // private final DigitalInput beamBreakSensor = new
    // DigitalInput(SubsystemConstants.Shooter.BEAM_BREAK_SENSOR_CHANNEL);

    private BaseStatusSignal shooterVelocity;
    private BaseStatusSignal shooterCurrent;
    private BaseStatusSignal shooterVoltage;
    private BaseStatusSignal shooterTemperature;

    // private BaseStatusSignal isBeamBreakTripped;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.kIndexer.kSpindexer.kP;
        config.Slot0.kI = SubsystemConstants.kIndexer.kSpindexer.kI;
        config.Slot0.kD = SubsystemConstants.kIndexer.kSpindexer.kD;
        config.Slot0.kS = SubsystemConstants.kIndexer.kSpindexer.kS;
        config.Slot0.kV = SubsystemConstants.kIndexer.kSpindexer.kV;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.kIndexer.kSpindexer.GEAR_RATIO;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kIndexer.kSpindexer.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kIndexer.kSpindexer.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.kIndexer.kSpindexer.MAX_SPEED_RPM;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.kIndexer.kSpindexer.SUPPLY_CURRENT_LIMIT;
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                SubsystemConstants.kIndexer.kSpindexer.PEAK_CURRENT_LIMIT;

        return config;
    }

    public SpindexerReal() {

        mainShooter.getConfigurator().apply(getLeaderConfig());

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        shooterVelocity = mainShooter.getVelocity();
        shooterCurrent = mainShooter.getSupplyCurrent();
        shooterVoltage = mainShooter.getSupplyVoltage();
        shooterTemperature = mainShooter.getDeviceTemp();
    }

    @Override
    public void goToRPM(double RPM) {
        DogLog.log("Subsystems/Indexer/Spindexer/setSpeed", RPM);
        mainShooter.setControl(velocityControl.withVelocity(RPM / 60.0));
        mainShooter.setControl(velocityControl.withVelocity(RPM / 60.0));
    }

    @Override
    public void stop() {
        mainShooter.setControl(dutyCycleControl.withOutput(0.0));
    }

    @Override
    public double getRPM() {
        return shooterVelocity.getValueAsDouble() * 60;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                shooterVelocity, shooterCurrent, shooterVoltage, shooterTemperature);
        DogLog.log(
                "Subsystems/Indexer/Spindexer/velocity", shooterVelocity.getValueAsDouble() * 60.0);
        DogLog.log("Subsystems/Indexer/Spindexer/current", shooterCurrent.getValueAsDouble());
        DogLog.log("Subsystems/Indexer/Spindexer/voltage", shooterVoltage.getValueAsDouble());
        DogLog.log(
                "Subsystems/Indexer/Spindexer/temperature", shooterTemperature.getValueAsDouble());
    }
}
