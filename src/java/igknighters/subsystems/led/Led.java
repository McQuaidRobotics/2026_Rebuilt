package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.led.driver.PWMDriver;
import igknighters.util.log.Log;
import wpilibExt.Tracer;

public class Led extends SubsystemBase {

    public final PWMDriver pwm1;

    public Led(int length, int numberOfStrips) {
        pwm1 = new PWMDriver(0, length, numberOfStrips, 60);
    }

    public void animate(AddressableLEDBuffer buffer) {
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Subsystems/LED/Animate", true);
        }
        pwm1.applyBuffer(buffer);
    }

    @Override
    public void periodic() {
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Subsystems/LED/Periodic", true);
        }
        Tracer.startTrace("LedPeriodic");
        pwm1.periodic();
        Tracer.endTrace();
    }
}
