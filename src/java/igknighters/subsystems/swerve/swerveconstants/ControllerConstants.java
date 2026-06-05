package igknighters.subsystems.swerve.swerveconstants;

import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;

public class ControllerConstants {
    public static final LerpTable TELEOP_TRANSLATION_AXIS_CURVE =
            new LerpTable(
                    new LerpTableEntry(0.0, 0.0),
                    new LerpTableEntry(0.12, 0.0), // deadzone
                    new LerpTableEntry(0.4, .3),
                    new LerpTableEntry(1.0, 1.0));

    public static final LerpTable TELEOP_ROTATION_AXIS_CURVE =
            new LerpTable(
                    new LerpTableEntry(0.0, 0.0),
                    new LerpTableEntry(0.12, 0.0), // deadzone
                    new LerpTableEntry(0.5, 0.3),
                    new LerpTableEntry(0.7, 0.6),
                    new LerpTableEntry(1.0, 1.0));
}
