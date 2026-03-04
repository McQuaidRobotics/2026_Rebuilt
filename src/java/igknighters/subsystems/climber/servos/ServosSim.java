package igknighters.subsystems.climber.servos;

import igknighters.constants.SubsystemConstants;
import igknighters.util.log.Log;

public class ServosSim extends Servos {

    private boolean deployed = false;

    @Override
    public void deploy() {
        deployed = true;
    }

    @Override
    public void retract() {
        deployed = false;
    }

    @Override
    public void periodic() {
        if (!SubsystemConstants.kClimber.kChainsaw.disableChainsawLogs) {
            Log.log("ROBOT/Subsystems/Climber/Servos/Deployed", deployed);
        }
    }
}
