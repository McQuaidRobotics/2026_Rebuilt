package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.ChainsawReal;
import igknighters.subsystems.climber.chainsaw.ChainsawSim;
import igknighters.subsystems.climber.servos.Servos;
import igknighters.subsystems.climber.servos.ServosReal;
import igknighters.subsystems.climber.servos.ServosSim;

public class Climber extends SubsystemBase {
    private final Chainsaw chainsaw;
    private final Servos servos;

    public Climber() {
        if (Robot.isReal()) {
            chainsaw = new ChainsawReal();
            servos = new ServosReal();
        } else {
            chainsaw = new ChainsawSim();
            servos = new ServosSim();
        }
    }

    public boolean isUp() {
        return chainsaw.isUp();
    }

    public boolean isDown() {
        return chainsaw.isDown();
    }

    public boolean isMiddle() {
        return chainsaw.isMiddle();
    }

    public void goUp() {
        chainsaw.goUp();
    }

    public void goDown() {
        chainsaw.goDown();
    }

    public void deployServo() {
        servos.deploy();
    }

    public void retractServo() {
        servos.retract();
    }

    public void stopChainsaw() {
        chainsaw.goToState(Chainsaw.ChainsawState.STOPPED);
    }

    public void goToState(ClimberState state) {
        chainsaw.goToState(state.chainsawState);
        if (state.servoDeployed) {
            servos.deploy();
        } else {
            servos.retract();
        }
        chainsaw.goToState(state.chainsawState);
    }

    public boolean isSensorHit() {
        return chainsaw.isSensorHit();
    }

    @Override
    public void periodic() {
        chainsaw.periodic();
        servos.periodic();
    }
}
