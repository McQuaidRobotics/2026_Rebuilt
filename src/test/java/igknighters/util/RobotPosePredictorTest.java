package igknighters.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RobotPosePredictorTest {
    @Test
    void argMaxArgMinReturnFirstExtreme() {
        // Must match the old Collections.max/min + indexOf behavior: first occurrence wins.
        double[] ts = {1.0, 5.0, 3.0, 5.0, 0.0, 0.0};
        assertEquals(1, RobotPosePredictor.argMax(ts));
        assertEquals(4, RobotPosePredictor.argMin(ts));
        assertEquals(0, RobotPosePredictor.argMax(new double[3]));
    }
}
