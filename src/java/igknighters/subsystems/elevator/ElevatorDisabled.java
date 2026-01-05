package igknighters.subsystems.elevator;

public class ElevatorDisabled extends Elevator {
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
