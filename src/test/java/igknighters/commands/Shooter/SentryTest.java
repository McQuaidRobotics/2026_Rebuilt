package igknighters.commands.Shooter;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import java.io.File;
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

    @Test
    void bearingHandlesYawAndUpsideDown() {
        var leftCam = new Rotation3d(0, 0, Math.PI / 2);
        assertEquals(90, Sentry.bearingDeg(leftCam, 0, 0), 1e-6);
        assertEquals(80, Sentry.bearingDeg(leftCam, 10, 0), 1e-6); // tx right -> less CCW
        // upside down, raw image: things on the right of the image are physically left
        var upsideDown = new Rotation3d(Math.PI, 0, 0);
        assertEquals(10, Sentry.bearingDeg(upsideDown, 10, 0), 1e-6);
        // upside down but Limelight flips the image: back to normal
        var flipped = new Rotation3d(Math.PI, 0, 0).plus(upsideDown);
        assertEquals(-10, Sentry.bearingDeg(flipped, 10, 0), 1e-6);
    }

    @Test
    void loadsPulledConfig() {
        var dir = new File("src/main/deploy/limelight-configs");
        var back = Sentry.loadCam(dir, "limelight-back", 0); // right-side up, yaw 180
        assertNotNull(back);
        assertTrue(back.fake());
        assertEquals(170, Math.abs(Sentry.bearingDeg(back.rot(), 10, 0)), 0.5);
        assertTrue(Sentry.bearingDeg(back.rot(), 10, 0) > 0); // facing back, tx right = robot left
        assertNull(Sentry.loadCam(dir, "limelight-back", 42));
        assertNull(Sentry.loadCam(dir, "no-such-cam", 0));
    }
}
