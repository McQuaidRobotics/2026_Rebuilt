package igknighters.subsystems.indexer.launcherRollers;

public class ExitRollersDisabled extends ExitRollers {
    @Override
    void setSpeedRPM(double rpm) {
        // Do nothing
    }

    @Override
    void setVoltage(double voltage) {
        // Do nothing
    }

    @Override
    double getSpeedRPM() {
        return 0.0;
    }

    @Override
    boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        return true;
    }

    @Override
    void periodic() {
        // Do nothing
    }
}
