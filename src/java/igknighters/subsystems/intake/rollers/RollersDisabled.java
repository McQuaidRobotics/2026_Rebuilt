package igknighters.subsystems.intake.rollers;

public class RollersDisabled extends Rollers {

    @Override
    public void goToSpeedRPM(double speed) {
        // Do nothing
    }

    @Override
    public double getSpeedRPM() {
        return 0.0;
    }

    @Override
    public void stop() {
        // Do nothing
    }

    @Override
    public void periodic() {
        // Do nothing
    }
}
