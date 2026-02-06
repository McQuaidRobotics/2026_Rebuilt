package igknighters.subsystems.led;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led extends SubsystemBase {

    public final PWMDriver pwm1;

    public Led(int length, int numberOfStrips) {
        pwm1 = new PWMDriver(0, length, numberOfStrips);
    }

    public void animate(AddressableLEDBuffer buffer) {

        pwm1.applyBuffer(buffer);
    }

    @Override
    public void periodic() {
        DogLog.log("Subsystems/LED/Periodic", true);
        Tracer.startTrace("LedPeriodic");
        pwm1.periodic();
        Tracer.endTrace();
    }
}
