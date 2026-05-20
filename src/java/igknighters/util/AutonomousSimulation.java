package igknighters.util;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.Robot;

/**
 * Utility to allow the robot program to be simulated with zero human interaction. This class
 * provides methods to spoof joystick inputs and manage the robot's state programmatically, allowing
 * an AI agent or automated script to control the robot.
 */
public class AutonomousSimulation {
    private final XboxControllerSim driverSim;
    private final Robot robot;

    public AutonomousSimulation(Robot robot) {
        this.robot = robot;
        // DriverController typically uses port 0
        this.driverSim = new XboxControllerSim(0);
    }

    public void setAutonomousRoutine(String routineName) {
        robot.autoChooser.select(routineName);
    }

    /** Initializes the simulation by setting the robot to a specific mode. */
    public void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
    }

    public void setAutonomous(boolean autonomous) {
        DriverStationSim.setAutonomous(autonomous);
        DriverStationSim.notifyNewData();
    }

    /**
     * Spoofs the left stick Y axis (typically forward/backward translation).
     *
     * @param value -1.0 to 1.0
     */
    public void setForward(double value) {
        driverSim.setLeftY(-value); // Inverted in DriverController
        DriverStationSim.notifyNewData();
    }

    /**
     * Spoofs the left stick X axis (typically strafe translation).
     *
     * @param value -1.0 to 1.0
     */
    public void setStrafe(double value) {
        driverSim.setLeftX(value);
        DriverStationSim.notifyNewData();
    }

    /**
     * Spoofs the right stick X axis (typically rotation).
     *
     * @param value -1.0 to 1.0
     */
    public void setRotation(double value) {
        driverSim.setRightX(-value); // Inverted in DriverController
        DriverStationSim.notifyNewData();
    }

    /**
     * Spoofs pressing a button on the controller.
     *
     * @param buttonId Use XboxControllerSim.set...Button() or similar
     */
    public void setButton(int button, boolean pressed) {
        // Generic way to set buttons if button ID is known
        // XboxControllerSim has explicit methods for most buttons
        switch (button) {
            case 1:
                driverSim.setAButton(pressed);
                break;
            case 2:
                driverSim.setBButton(pressed);
                break;
            case 3:
                driverSim.setXButton(pressed);
                break;
            case 4:
                driverSim.setYButton(pressed);
                break;
            case 5:
                driverSim.setLeftBumper(pressed);
                break;
            case 6:
                driverSim.setRightBumper(pressed);
                break;
            case 7:
                driverSim.setBackButton(pressed);
                break;
            case 8:
                driverSim.setStartButton(pressed);
                break;
        }
        DriverStationSim.notifyNewData();
    }

    /**
     * Updates the simulation by one "tick". This is useful if you want to run the robot loop
     * manually in a test environment.
     */
    public void tick() {
        HAL.initialize(500, 0);
        CommandScheduler.getInstance().run();
    }

    /** Resets all inputs to zero. */
    public void resetInputs() {
        driverSim.setLeftX(0);
        driverSim.setLeftY(0);
        driverSim.setRightX(0);
        driverSim.setRightY(0);
        driverSim.setAButton(false);
        driverSim.setBButton(false);
        driverSim.setXButton(false);
        driverSim.setYButton(false);
        driverSim.setLeftBumper(false);
        driverSim.setRightBumper(false);
        DriverStationSim.notifyNewData();
    }

    public void tearDown() {
        robot.close();
    }
}
