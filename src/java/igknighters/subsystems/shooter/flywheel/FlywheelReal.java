package igknighters.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.Robot;
import igknighters.util.log.Log;

public class FlywheelReal extends Flywheel {

    private final TalonFX mainShooter;
    private final TalonFX followerShooter;

    // private final MotionMagicVelocityVoltage velocityControl = new

    private boolean isBeingControlledActivly = false;
    // MotionMagicVelocityVoltage(0.0);

    private final MotionMagicVelocityVoltage velocityControl;
    // private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
    //         new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Robot.consts.shooter().kFlywheels().kP();
        config.Slot0.kI = Robot.consts.shooter().kFlywheels().kI();
        config.Slot0.kD = Robot.consts.shooter().kFlywheels().kD();
        config.Slot0.kS = Robot.consts.shooter().kFlywheels().kS();
        config.Slot0.kV = Robot.consts.shooter().kFlywheels().kV();

        config.Feedback.SensorToMechanismRatio = Robot.consts.shooter().kFlywheels().GEAR_RATIO();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotionMagic.MotionMagicJerk =
                Robot.consts.shooter().kFlywheels().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicAcceleration =
                Robot.consts.shooter().kFlywheels().MAX_ACCELERATION_RPM();
        config.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.shooter().kFlywheels().MAX_SPEED_RPM();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                Robot.consts.shooter().kFlywheels().SUPPLY_CURRENT_LIMIT();
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse

        return config;
    }

    public FlywheelReal() {

        mainShooter =
                new TalonFX(
                        Robot.consts.shooter().kFlywheels().LEADER_MOTOR_ID(),
                        Robot.consts.shooter().kCANBUS());
        followerShooter =
                new TalonFX(
                        Robot.consts.shooter().kFlywheels().FOLLOWER_MOTOR_ID(),
                        Robot.consts.shooter().kCANBUS());

        mainShooter.getConfigurator().apply(getLeaderConfig());
        followerShooter.setControl(
                new Follower(mainShooter.getDeviceID(), MotorAlignmentValue.Opposed));

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);
    }

    @Override
    public void setSpeed(AngularVelocity speedRPM) {
        if (!Robot.consts.shooter().kFlywheels().disableFlywheelsLogs()) {
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
        if (!Robot.consts.shooter().kFlywheels().disableFlywheelsLogs()) {
            Log.log(
                    "ROBOT/Subsystems/Shooter/Flywheels/being controlled",
                    isBeingControlledActivly);
            Log.logMotor("ROBOT/Subsystems/Shooter/Flywheels/Motor", mainShooter);
        }

        isBeingControlledActivly = false;
    }
}
