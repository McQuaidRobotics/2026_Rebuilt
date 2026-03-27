package igknighters.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.List;

public class LEDCommands {

    public record LEDSection(int index, int offset, LEDPattern pattern, int length, String name) {}

    // Internal record to cache our pre-calculated views
    private record PreparedSection(LEDPattern pattern, AddressableLEDBufferView view) {}

    /** Main run command using varargs to avoid list conversions. */
    public static Command run(Led led, LEDSection... sections) {
        final AddressableLEDBuffer slate = new AddressableLEDBuffer(led.pwm1.length);
        final LEDPattern eraser = LEDPattern.solid(Color.kBlack);

        // 1. Pre-calculate static variables outside the execution loop
        int stripLength = led.pwm1.length / led.pwm1.numberOfStrips;

        // 2. Pre-create the views so we aren't allocating memory every 20ms
        List<PreparedSection> preparedSections = new ArrayList<>(sections.length);
        List<String> names = new ArrayList<>(sections.length);

        for (LEDSection section : sections) {
            int stripStart = section.index() * stripLength;
            int stripEnd = stripStart + stripLength - 1;

            int startView = MathUtil.clamp(stripStart + section.offset(), stripStart, stripEnd);
            int endView =
                    MathUtil.clamp(
                            stripStart + section.offset() + section.length() - 1,
                            stripStart,
                            stripEnd);

            // Create the view ONCE during command construction
            AddressableLEDBufferView view = slate.createView(startView, endView);
            preparedSections.add(new PreparedSection(section.pattern(), view));
            names.add(section.name());
        }

        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Subsystems/LED/Run/Sections", sections.length);
        }

        return Commands.startRun(
                        () -> {
                            // Init: Run once when the command starts
                            eraser.applyTo(slate);
                        },
                        () -> {
                            // Execute: Runs every 20ms. Extremely fast, zero allocations.
                            for (PreparedSection ps : preparedSections) {
                                ps.pattern().applyTo(ps.view());
                            }

                            led.animate(slate);
                            LedUtil.logBuffer("fullPattern", led, slate);
                        },
                        led)
                .withName("SplitLed(" + sections.length + ", " + names + ")");
    }

    /** Overload for passing a List of sections. */
    public static Command run(Led led, List<LEDSection> sections) {
        return run(led, sections.toArray(new LEDSection[0]));
    }

    /** Overload for applying a single pattern to all strips. */
    public static Command run(Led led, LEDPattern pattern) {
        int stripLength = led.pwm1.length / led.pwm1.numberOfStrips;
        LEDSection[] sections = new LEDSection[led.pwm1.numberOfStrips];

        for (int i = 0; i < led.pwm1.numberOfStrips; i++) {
            String name = "full led strip " + (i + 1);
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/Subsystems/LED/Run/AddingSection", name);
            }
            sections[i] = new LEDSection(i, 0, pattern, stripLength, name);
        }

        return run(led, sections);
    }
}
