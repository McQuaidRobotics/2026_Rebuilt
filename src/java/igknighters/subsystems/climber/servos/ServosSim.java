package igknighters.subsystems.climber.servos;

import dev.doglog.DogLog;

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
        DogLog.log("Subsystems/Climber/Servos/Deployed", deployed);
    }
}
