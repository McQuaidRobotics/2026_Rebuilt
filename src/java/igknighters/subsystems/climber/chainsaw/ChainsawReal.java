package igknighters.subsystems.climber.chainsaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kClimber;
import igknighters.util.log.Log;

public class ChainsawReal extends Chainsaw {

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private ChainsawState state = ChainsawState.STOPPED;
    private boolean goUpToMiddle = false;

    private final DigitalInput bumperSensor =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.BUMPER_SENSOR_ID);

    private final DigitalInput upperLimitSwitch =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_SENSOR_ID);
    private final DigitalInput lowerLimitSwitch =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_SENSOR_ID);
    private final DigitalInput middleLimitSwitch =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.MIDDLE_HEIGHT_SENSOR_ID);

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX leftMotor;

    @Override
    public boolean isDown() {
        return !lowerLimitSwitch.get();
    }

    @Override
    public boolean isMiddle() {
        return !middleLimitSwitch.get();
    }

    @Override
    public void goToState(ChainsawState state) {
        if (state == ChainsawState.GOING_TO_MIDDLE) {
            goUpToMiddle = isDown();
        }
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
        leftMotor =
                new TalonFX(SubsystemConstants.kClimber.kChainsaw.LEFT_MOTOR_ID, kClimber.CANBUS);

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
        } else if (state == ChainsawState.GOING_TO_MIDDLE) {
            if (isMiddle()) {
                state = ChainsawState.STOPPED;
                output = 0.0;
            } else {
                if (isDown()) {
                    goUpToMiddle = true;
                } else if (isUp()) {
                    goUpToMiddle = false;
                }
                output = goUpToMiddle ? 0.3 : -0.5;
            }
        } else {
            output = 0.0;
        }

        leftMotor.setControl(dutyCycleControl.withOutput(output));

        if (!SubsystemConstants.kClimber.kChainsaw.disableChainsawLogs) {
            Log.log(
                    "Subsystems/Climber/Inches",
                    armPosition.getValueAsDouble()
                            * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES);
            Log.log("Subsystems/Climber/Is Up", isUp());
            Log.log("Subsystems/Climber/Is Middle", isMiddle());
            Log.log("Subsystems/Climber/Is Down", isDown());
            Log.log("Subsystems/Climber/Sensor Hit", isSensorHit());
            Log.log("Subsystems/Climber/State", state.toString());
            Log.log("Subsystems/Climber/Current", armCurrent.getValueAsDouble());
        }
    }
}
