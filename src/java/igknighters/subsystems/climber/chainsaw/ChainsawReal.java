package igknighters.subsystems.climber.chainsaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.constants.SubsystemConstants;

public class ChainsawReal extends Chainsaw {

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private ChainsawState state = ChainsawState.STOPPED;

    private final DigitalInput bumperSensor =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.BUMPER_SENSOR_ID);

    private final DigitalInput upperLimitSwitch =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_SENSOR_ID);
    private final DigitalInput lowerLimitSwitch =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_SENSOR_ID);

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX leftMotor;

    @Override
    public boolean isDown() {
        return !lowerLimitSwitch.get();
    }

    @Override
    public void goToState(ChainsawState state) {
        this.state = state;
    }

    @Override
    public boolean isUp() {
        return !upperLimitSwitch.get();
    }

    @Override
    public void goDown() {
        state = ChainsawState.GOING_DOWN;
    }

    @Override
    public void goUp() {
        state = ChainsawState.GOING_UP;
    }

    @Override
    public boolean isSensorHit() {
        return !bumperSensor.get();
    }

    public ChainsawReal() {
        leftMotor = new TalonFX(SubsystemConstants.kClimber.kChainsaw.LEFT_MOTOR_ID);

        armPosition = leftMotor.getPosition();
        armCurrent = leftMotor.getStatorCurrent();

        leftMotor.getConfigurator().apply(arm1Config());
    }

    public TalonFXConfiguration arm1Config() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit =
                SubsystemConstants.kClimber.kChainsaw.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kClimber.kChainsaw.SUPPLY_CURRENT_LIMIT;

        return config;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armPosition, armCurrent);

        double output = 0.0;
        if (state == ChainsawState.GOING_UP) {
            if (isUp()) {
                state = ChainsawState.STOPPED;
                output = 0.0;
            } else {
                output = 0.3; // 30% power up, adjust as needed
            }
        } else if (state == ChainsawState.GOING_DOWN) {
            if (isDown()) {
                state = ChainsawState.STOPPED;
                output = 0.0;
            } else {
                output = -0.5; // 50% power down, adjust as needed
            }
        } else {
            output = 0.0;
        }

        leftMotor.setControl(dutyCycleControl.withOutput(output));

        DogLog.log(
                "Subsystems/Climber/Inches",
                armPosition.getValueAsDouble()
                        * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES);
        DogLog.log("Subsystems/Climber/Is Up", isUp());
        DogLog.log("Subsystems/Climber/Is Down", isDown());
        DogLog.log("Subsystems/Climber/Sensor Hit", isSensorHit());
        DogLog.log("Subsystems/Climber/State", state.toString());
        DogLog.log("Subsystems/Climber/Current", armCurrent.getValueAsDouble());
    }
}
