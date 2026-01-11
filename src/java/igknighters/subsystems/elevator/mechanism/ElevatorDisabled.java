package igknighters.subsystems.elevator.mechanism;

public class ElevatorDisabled extends ElevatorMechanism {
    @Override
    public void setHeight(double height) {
        // Do nothing
    }

    @Override
    public double getHeight() {
        return 0;
    }

    @Override
    public boolean isAt(double height, double tolerance) {
        return true;
    }

    @Override
    public void moveToHeight(double height) {
        // Do nothing
    }
}
