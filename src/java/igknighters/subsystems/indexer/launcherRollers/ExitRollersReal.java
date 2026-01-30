package igknighters.subsystems.indexer.launcherRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import igknighters.constants.SubsystemConstants;

public class ExitRollersReal extends ExitRollers {
    private final TalonFX exitRollerMotor =
            new TalonFX(SubsystemConstants.kIndexer.kExitRollers.LEADER_MOTOR_ID);

    private final MotionMagicVelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private BaseStatusSignal shooterVelocity;
    private BaseStatusSignal shooterCurrent;
    private BaseStatusSignal shooterVoltage;
    private BaseStatusSignal shooterTemperature;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.kIndexer.kExitRollers.kP;
        config.Slot0.kI = SubsystemConstants.kIndexer.kExitRollers.kI;
        config.Slot0.kD = SubsystemConstants.kIndexer.kExitRollers.kD;
        config.Slot0.kS = SubsystemConstants.kIndexer.kExitRollers.kS;
        config.Slot0.kV = SubsystemConstants.kIndexer.kExitRollers.kV;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.kIndexer.kExitRollers.GEAR_RATIO;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kIndexer.kExitRollers.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kIndexer.kExitRollers.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kIndexer.kExitRollers.MAX_SPEED_RPM;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kIndexer.kExitRollers.SUPPLY_CURRENT_LIMIT;
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                SubsystemConstants.kIndexer.kExitRollers.PEAK_CURRENT_LIMIT;

        return config;
    }

    public ExitRollersReal() {

        exitRollerMotor.getConfigurator().apply(getLeaderConfig());

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        shooterVelocity = exitRollerMotor.getVelocity();
        shooterCurrent = exitRollerMotor.getSupplyCurrent();
        shooterVoltage = exitRollerMotor.getSupplyVoltage();
        shooterTemperature = exitRollerMotor.getDeviceTemp();
    }

    @Override
    public void setSpeedRPM(double speedRpm) {
        DogLog.log("Subsystems/Shooter/Rollers/setSpeed", speedRpm);
        exitRollerMotor.setControl(velocityControl.withVelocity(speedRpm / 60.0));
    }

    @Override
    public void setVoltage(double voltage) {
        exitRollerMotor.setControl(dutyCycleControl.withOutput(voltage / 12.0));
    }

    @Override
    boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        double currentRPM = getSpeedRPM();
        return Math.abs(currentRPM - targetRPM) <= toleranceRPM;
    }

    @Override
    public double getSpeedRPM() {
        return shooterVelocity.getValueAsDouble();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                shooterVelocity, shooterCurrent, shooterVoltage, shooterTemperature);
        DogLog.log(
                "Subsystems/Shooter/Rollers/velocity", shooterVelocity.getValueAsDouble() * 60.0);
        DogLog.log("Subsystems/Shooter/Rollers/current", shooterCurrent.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Rollers/voltage", shooterVoltage.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Rollers/temperature", shooterTemperature.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Rollers/periodicing", true);
    }
}
