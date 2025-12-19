package igknighters.subsystems.led.driver;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import monologue.Logged;

public class PWMDriver implements Logged {

    private final AddressableLED led;
    private final AddressableLEDBuffer previousBuffer;
    public final int length;
    public final int numberOfStrips;

    // private final AddressableLEDBuffer buffer;

    public PWMDriver(int port, int length, int numberOfStrips) {
        led = new AddressableLED(port);
        this.length = length;
        this.numberOfStrips = numberOfStrips;
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
        boolean newBuffer = false;
        if (appliedBuffer == previousBuffer) {
            newBuffer = false;
        } else {
            newBuffer = true;
            led.setData(appliedBuffer);
        }
        log("new buffer", newBuffer);
    }

    public void periodic() {}
}
