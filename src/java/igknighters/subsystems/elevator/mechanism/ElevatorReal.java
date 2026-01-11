package igknighters.subsystems.elevator.mechanism;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.elevator.ElevatorVisualizer;

public class ElevatorReal extends ElevatorMechanism {

    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;
    private final MotionMagicTorqueCurrentFOC positionControl = new MotionMagicTorqueCurrentFOC(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final StatusSignal<Angle> elevatorHeight;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Temperature> elevatorTemperature;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitchTriggered;
    private double goal = 0;
    ElevatorVisualizer visualizer = new ElevatorVisualizer();

    public ElevatorReal() {
        elevatorLeader = new TalonFX(SubsystemConstants.Elevator.LEADER_MOTOR_ID);
        elevatorFollower = new TalonFX(SubsystemConstants.Elevator.FOLLOWER_MOTOR_ID);
        elevatorHeight = elevatorLeader.getPosition();
        elevatorVelocity = elevatorLeader.getVelocity();
        elevatorTemperature = elevatorLeader.getDeviceTemp();
        reverseLimitSwitchTriggered = elevatorLeader.getReverseLimit();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.Elevator.kP;
        config.Slot0.kI = SubsystemConstants.Elevator.kI;
        config.Slot0.kD = SubsystemConstants.Elevator.kD;
        config.Slot0.kS = SubsystemConstants.Elevator.kS;
        config.Slot0.kG = SubsystemConstants.Elevator.kG;
        config.Slot0.kV = SubsystemConstants.Elevator.kV;
        config.Feedback.SensorToMechanismRatio = SubsystemConstants.Elevator.GEAR_RATIO;
        // TODO NEED TO ADD THE REMOTE SENSOR IN HERE BUT IDK HOW
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
                SubsystemConstants.Elevator.MIN_HEIGHT_METERS
                        / (2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS);
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID =
                SubsystemConstants.Elevator.REVERSE_LIMIT_REMOTE_SENSOR_ID;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.Elevator.MAX_HEIGHT_METERS
                        / (2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS);
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                        / (2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS);
        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.Elevator.MAX_SPEED_METERS_PER_SECOND
                        / (2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS);

        elevatorLeader.getConfigurator().apply(config);
        elevatorFollower.setControl(
                new Follower(elevatorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void moveToHeight(double height) {
        goal = height;
        double rotations =
                height / (2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS);
        elevatorLeader.setControl(positionControl.withPosition(rotations));
    }

    @Override
    public void setHeight(double height) {
        double rotations =
                height / (2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS);
        elevatorLeader.setPosition(rotations);
    }

    @Override
    public double getHeight() {
        double rotations = elevatorHeight.getValue().in(Rotation);
        return rotations * 2.0 * Math.PI * SubsystemConstants.Elevator.DRUM_RADIUS_METERS;
    }

    @Override
    public boolean isAt(double height, double tolerance) {
        return Math.abs(getHeight() - height) <= tolerance;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                elevatorHeight, elevatorTemperature, elevatorVelocity, reverseLimitSwitchTriggered);

        DogLog.log("Subsystems/Elevator/HeightMeters", elevatorHeight.getValue());
        DogLog.log("Subsystems/Elevator/VelocityMetersPerSecond", elevatorVelocity.getValue());
        DogLog.log("Subsystems/Elevator/TemperatureCelsius", elevatorTemperature.getValue());
        DogLog.log(
                "Subsystems/Elevator/ReverseLimitSwitchTriggered",
                reverseLimitSwitchTriggered.getValue());
        visualizer.update(getHeight(), goal);
    }
}
