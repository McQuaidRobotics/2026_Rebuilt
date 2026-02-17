package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.ChainsawDisabled;
import igknighters.subsystems.climber.chainsaw.ChainsawSim;
import igknighters.subsystems.climber.servos.Servos;
import igknighters.subsystems.climber.servos.ServosDisabled;
import igknighters.subsystems.climber.servos.ServosSim;

public class Climber extends SubsystemBase {
    private Chainsaw chainsaw;
    private Servos servos;
    private boolean usingRealSensor;

    public Climber() {
        if (Robot.isReal()) {
            chainsaw = new ChainsawDisabled();
            servos = new ServosDisabled();
            usingRealSensor = true;
        } else {
            chainsaw = new ChainsawSim();
            servos = new ServosSim();
            usingRealSensor = false;
        }
    }

    public boolean isUp() {
        return chainsaw.isUp();
    }

    public boolean isDown() {
        return chainsaw.isDown();
    }

    public void goUp() {
        chainsaw.goUp();
    }

    public void goDown() {
        chainsaw.goDown();
    }

    public void goToState(ClimberState state){
        chainsaw.goToState(state.chainsawState);
    }

    public boolean isSensorHit() {
        return chainsaw.isSensorHit();
    }

    @Override
    public void periodic() {
        chainsaw.periodic();
    }
}
