package igknighters.subsystems.led.driver;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import igknighters.Robot;
import igknighters.util.log.Log;
import monologue.Logged;

public class PWMDriver implements Logged {

    private final AddressableLED led;
    private AddressableLEDBuffer previousBuffer;
    public final int length;
    public final int numberOfStrips;
    public final int endOfStrip1;

    // private final AddressableLEDBuffer buffer;

    public PWMDriver(int port, int length, int numberOfStrips, int endOfStrip1) {
        led = new AddressableLED(port);
        this.length = length;
        this.numberOfStrips = numberOfStrips;
        this.endOfStrip1 = endOfStrip1;
        led.setLength(length);
        led.start();
        previousBuffer = new AddressableLEDBuffer(length);
    }

    /**
     * will apply a buffer to the LED if its a new one to take up as little resources as possible
     *
     * @param appliedBuffer
     */
    public void applyBuffer(AddressableLEDBuffer appliedBuffer) {
        // OK SO THE REASON THAT RAINBOW DID NOT DISPLAY DYNAMICALLY
        // WAS BECAUSE OF THIS IF. I MADE IT APPLY EVEN WITH SAME BUFFER
        // THIS FIXED IT HOWEVER I AM NOT SURE HOW RESOURSE INTENSIVE IT IS
        // TO APPLY BUFFER EACH CYCLE
        // IF CRAZY LOOP OVER-RUNS CHECK THIS
        boolean newBuffer = false;
        if (appliedBuffer == previousBuffer) {
            newBuffer = false;
            led.setData(appliedBuffer);
        } else {
            newBuffer = true;
            previousBuffer = appliedBuffer;
            led.setData(appliedBuffer);
        }
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Subsystems/LED/New Buffer Applied", newBuffer);
        }
    }

    public void periodic() {}
}
