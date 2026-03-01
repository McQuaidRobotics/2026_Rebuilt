package igknighters.subsystems.climber.servos;


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
        Log.log("Subsystems/Climber/Servos/Deployed", deployed);
    }
}
