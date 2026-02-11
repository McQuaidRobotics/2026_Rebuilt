package igknighters.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import java.util.ArrayList;
import java.util.List;

public class LEDCommands {

    public record LEDSection(int index, int offset, LEDPattern pattern, int length, String name) {}

    public static Command run(
            Led led,
            List<Integer> offsets,
            List<LEDPattern> patterns,
            List<Integer> lengths,
            List<Integer> index,
            List<String> names) {
        final AddressableLEDBuffer slate = new AddressableLEDBuffer(led.pwm1.length);
        final LEDPattern eraser = LEDPattern.solid(Color.kBlack);

        return Commands.startRun(
                        () -> {
                            eraser.applyTo(slate);
                        },
                        () -> {
                            if (offsets.size() != patterns.size()) {
                                DriverStation.reportError(
                                        "incorect lengths on offsets and patterns LED COMMANDS ",
                                        false);
                                System.out.println(
                                        "incorect lengths on offsets and patterns LED COMMANDS ");
                                return;
                            }
                            // System.out.println("Running SplitLed Command");

                            for (int i = 0; i < patterns.size(); i++) {
                                int stripIndex = index.get(i);
                                int stripLength = led.pwm1.length / led.pwm1.numberOfStrips;
                                int stripStart = stripIndex * stripLength;
                                int stripEnd = stripStart + stripLength - 1;

                                AddressableLEDBufferView controlledZone =
                                        slate.createView(
                                                MathUtil.clamp(
                                                        stripStart + offsets.get(i),
                                                        stripStart,
                                                        stripEnd),
                                                MathUtil.clamp(
                                                        stripStart
                                                                + offsets.get(i)
                                                                + lengths.get(i)
                                                                - 1,
                                                        stripStart,
                                                        stripEnd));
                                patterns.get(i).applyTo(controlledZone);
                            }
                            led.animate(slate);
                            LedUtil.logBuffer("fullPattern", led, slate);
                        })
                .withName(
                        "SplitLed("
                                + patterns.size()
                                + ", "
                                + offsets.size()
                                + ", "
                                + index
                                + ", "
                                + names
                                + ")");
    }

    public static Command run(Led led, LEDSection... ledSections) {
        List<Integer> offsets = new ArrayList<Integer>();
        List<LEDPattern> patterns = new ArrayList<LEDPattern>();
        List<Integer> lengths = new ArrayList<Integer>();
        List<Integer> indexes = new ArrayList<Integer>();
        List<String> names = new ArrayList<String>();

        for (int i = 0; i < ledSections.length; i++) {
            LEDSection ledSection = ledSections[i];
            offsets.add(ledSection.offset);
            patterns.add(ledSection.pattern);
            lengths.add(ledSection.length);
            indexes.add(ledSection.index);
            names.add(ledSection.name);
        }
        DogLog.log("Subsystems/LED/Run/Sections", ledSections.length);

        return run(led, offsets, patterns, lengths, indexes, names);
    }

    public static Command run(Led led, LEDPattern pattern) {
        List<LEDSection> sections = new ArrayList<>();
        int stripLength = led.pwm1.length / led.pwm1.numberOfStrips;
        for (int i = 0; i < led.pwm1.numberOfStrips; i++) {
            DogLog.log("Subsystems/LED/Run/AddingSection", "full led strip " + (i + 1));
            sections.add(new LEDSection(i, 0, pattern, stripLength, "full led strip " + (i + 1)));
        }
        return run(led, sections.toArray(new LEDSection[0]));
    }
}
