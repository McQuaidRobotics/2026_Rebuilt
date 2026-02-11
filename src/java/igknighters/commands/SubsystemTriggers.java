package igknighters.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.constants.AbleToShootSharedState;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import java.util.function.BooleanSupplier;

public class SubsystemTriggers {
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger autonomous = RobotModeTriggers.autonomous();
    private final Trigger teleop = RobotModeTriggers.teleop();

    public static Trigger falseOnce() {
        return new Trigger(
                new BooleanSupplier() {
                    boolean ret = false;

                    public boolean getAsBoolean() {
                        try {
                            return ret;
                        } finally {
                            ret = true;
                        }
                    }
                    ;
                });
    }

    public void SetupTriggers(Led led) {
        falseOnce()
                .and(disabled)
                .whileTrue(
                        
                                LEDCommands.run(led, LEDPattern.solid(Color.kRed))
                                        .ignoringDisable(true)
                                        .withName("DisabledRed"));

        autonomous.whileTrue(
                LEDCommands.run(led, LedUtil.makeRainbow(255, 256))
                        .ignoringDisable(true)
                        .withName("AutoRainbow"));

        teleop.whileTrue(
                LEDCommands.run(led, LEDPattern.solid(Color.kGreen))
                        .ignoringDisable(true)
                        .withName("TeleopGreen"));

        // Get the AbleToShootSharedState singleton
        AbleToShootSharedState ableToShootState = AbleToShootSharedState.getInstance();

        // Bind LED commands to the canShootTrigger
        ableToShootState
                .canShootTrigger()
                .whileTrue(LEDCommands.run(led, LEDPattern.solid(Color.kYellow)));
        ableToShootState
                .canShootTrigger()
                .whileFalse(LEDCommands.run(led, LEDPattern.solid(Color.kPurple)));
    }
}
