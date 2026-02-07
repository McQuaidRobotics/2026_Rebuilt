package igknighters.subsystems.indexer.launcherRollers;

public class ExitRollersDisabled extends ExitRollers {
    @Override
    public void setSpeedRPM(double rpm) {
        // Do nothing
    }

    @Override
    public void setVoltage(double voltage) {
        // Do nothing
    }

    @Override
    public double getSpeedRPM() {
        return 0.0;
    }

    @Override
    public boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        return true;
    }

    @Override
    public void periodic() {
        // Do nothing
    }
}
