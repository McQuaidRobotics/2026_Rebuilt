package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class TurretSim extends Turret {
    private TalonFX motor = new TalonFX(SubsystemConstants.Turret.MOTOR_ID);
    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);
    private DCMotorSim m_motorSimModel =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60Foc(1),
                            0.001,
                            SubsystemConstants.Turret.GEAR_RATIO),
                    DCMotor.getKrakenX60Foc(1));

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    SubsystemConstants.Turret.kS,
                    SubsystemConstants.Turret.kV,
                    SubsystemConstants.Turret.kA);

    private SingleJointedArmSim turretSim;

    private final TalonFXConfiguration turretConfiguration() {
        var cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = SubsystemConstants.Turret.kP;
        cfg.Slot0.kD = SubsystemConstants.Turret.kD;
        cfg.Slot0.kS = SubsystemConstants.Turret.kS;
        cfg.Slot0.kV = SubsystemConstants.Turret.kV;
        cfg.Slot0.kA = SubsystemConstants.Turret.kA;

        cfg.Feedback.RotorToSensorRatio = SubsystemConstants.Turret.GEAR_RATIO;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = SubsystemConstants.Turret.CANCODER_ID;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.Turret.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.Turret.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;

        cfg.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.Turret.MAX_SPEED_RPM * Conv.RPM_TO_RPS;
        cfg.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.Turret.MAX_ACCELERATION_RPM * Conv.RPM_TO_RPS;

        cfg.CurrentLimits.StatorCurrentLimit = SubsystemConstants.Turret.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.Turret.SUPPLY_CURRENT_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return cfg;
    }

    public TurretSim() {
        turretSim =
                new SingleJointedArmSim(
                        LinearSystemId.createSingleJointedArmSystem(
                                DCMotor.getKrakenX60(1),
                                SubsystemConstants.Shooter.MOMENT_OF_INERTIA_KG_M2,
                                SubsystemConstants.Turret.GEAR_RATIO),
                        DCMotor.getKrakenX60(1),
                        SubsystemConstants.Turret.GEAR_RATIO,
                        .2, // this is not a number that i know it is mainly for gravity sim which
                        // we dont need
                        SubsystemConstants.Turret.MIN_ANGLE_DEGREES / 360 * 2 * Math.PI,
                        SubsystemConstants.Turret.MAX_ANGLE_DEGREES / 360 * 2 * Math.PI,
                        false,
                        0.0,
                        0.0);

        motor.getConfigurator().apply(turretConfiguration());
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        super.degrees = angleDegrees;
        turretSim.setState(angleDegrees / 360 * 2 * Math.PI, 0.0);
        motor.setPosition(degrees / 360.0);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        motor.setControl(positionControl.withPosition(angleDegrees / 360));
    }

    @Override
    public double getAngleDegrees() {
        return motor.getPosition().getValueAsDouble() * 360.0;
    }

    @Override
    public void periodic() {
        var talonFXSim = motor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(
                m_motorSimModel.getAngularPosition().times(SubsystemConstants.Turret.GEAR_RATIO));
        talonFXSim.setRotorVelocity(
                m_motorSimModel.getAngularVelocity().times(SubsystemConstants.Turret.GEAR_RATIO));
    }
}
