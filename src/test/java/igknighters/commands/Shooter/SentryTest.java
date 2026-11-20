package igknighters.commands.Shooter;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

class SentryTest {
    @Test
    void neverAimsBehind() {
        assertEquals(Sentry.FRONT_DEG + Sentry.ARC_HALF_DEG, Sentry.clampToArc(170));
        assertEquals(Sentry.FRONT_DEG - Sentry.ARC_HALF_DEG, Sentry.clampToArc(-170));
        assertFalse(Sentry.inArc(Sentry.FRONT_DEG + 120));
        assertFalse(Sentry.inArc(Sentry.FRONT_DEG - 180));
        assertTrue(Sentry.inArc(Sentry.FRONT_DEG + 10));
    }

    @Test
    void picksTargets() {
        var seen = Map.of(33, 30.0, 34, -20.0, 35, 5.0); // CCW+ so 33 is leftmost
        assertEquals(34, Sentry.pickSide(seen, 1.0));
        assertEquals(33, Sentry.pickSide(seen, -1.0));
        assertNotEquals(35, Sentry.pickOther(seen, 35, new Random(1)));
        assertEquals(33, Sentry.pickOther(Map.of(33, 0.0), 33, new Random(1)));
        assertEquals(-1, Sentry.pickOther(Map.of(), 33, new Random(1)));
    }
}
