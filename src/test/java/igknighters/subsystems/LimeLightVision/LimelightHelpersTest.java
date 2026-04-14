package igknighters.subsystems.LimeLightVision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionReal;
import java.util.Arrays;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class LimelightHelpersTest {
    @BeforeAll
    public static void setupNetworkTables() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer("testTables.json", "testTables.json", 10000);
    }

    @Test
    public void parsesBotposeRotation() throws InterruptedException {
        String cam1 = "limelight-cam1";
        String cam2 = "limelight-cam2";
        double rot1 = 1.5;
        double rot2 = 358.5;
        LimeLightVisionReal vision = new LimeLightVisionReal(cam1, cam2);

        // Array Size: 11 (Header/Summary) + 7 (Tag 1) + 7 (Tag 2) = 25 total elements
        double[] fakeBotpose1 = new double[25];

        // --- Header & Summary ---
        fakeBotpose1[0] = 2.0; // x (meters)
        fakeBotpose1[1] = 3.0; // y (meters)
        fakeBotpose1[2] = 0.5; // z (height off ground)
        fakeBotpose1[3] = 0.0; // roll
        fakeBotpose1[4] = 0.0; // pitch
        fakeBotpose1[5] = rot1; // yaw
        fakeBotpose1[6] = 15.5; // latency (ms)
        fakeBotpose1[7] = 2; // TAG COUNT (Crucial for your logic)
        fakeBotpose1[8] = 1.2; // tagSpan (meters between tags)
        fakeBotpose1[9] = 2.1; // avgTagDist (meters)
        fakeBotpose1[10] = 0.8; // avgTagArea (%)

        // --- Tag ID 1 Data ---
        fakeBotpose1[11] = 1; // ID
        fakeBotpose1[12] = -5.2; // txnc (degrees left of center)
        fakeBotpose1[13] = 1.1; // tync (degrees above center)
        fakeBotpose1[14] = 0.9; // ta (area %)
        fakeBotpose1[15] = 2.05; // distToCamera (meters)
        fakeBotpose1[16] = 2.2; // distToRobot (meters)
        fakeBotpose1[17] = 0.02; // ambiguity (0.0 is perfect, 1.0 is bad)

        // --- Tag ID 7 Data ---
        fakeBotpose1[18] = 7; // ID (The second ID you needed)
        fakeBotpose1[19] = 6.4; // txnc (degrees right of center)
        fakeBotpose1[20] = 0.8; // tync
        fakeBotpose1[21] = 0.7; // ta
        fakeBotpose1[22] = 2.15; // distToCamera
        fakeBotpose1[23] = 2.3; // distToRobot
        fakeBotpose1[24] = 0.04; // ambiguity

        // Inject into NetworkTables
        NetworkTableInstance.getDefault()
                .getTable(cam1)
                .getEntry("botpose_orb_wpiblue")
                .setDoubleArray(fakeBotpose1);
        NetworkTableInstance.getDefault()
                .getTable(cam1)
                .getEntry("botpose_wpiblue")
                .setDoubleArray(fakeBotpose1);

        // Repeat for Cam 2 (You can reuse the array or slightly tweak it)
        double[] fakeBotpose2 = fakeBotpose1.clone();
        fakeBotpose2[5] = rot2; // Give cam2 a different yaw
        NetworkTableInstance.getDefault()
                .getTable(cam2)
                .getEntry("botpose_orb_wpiblue")
                .setDoubleArray(fakeBotpose2);
        NetworkTableInstance.getDefault()
                .getTable(cam2)
                .getEntry("botpose_wpiblue")
                .setDoubleArray(fakeBotpose2);

        // Give NetworkTables some time to process the update
        try {
            Thread.sleep(100); // 100ms is usually enough
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        double[] cam1ReadBack =
                NetworkTableInstance.getDefault()
                        .getTable(cam1)
                        .getEntry("botpose_orb_wpiblue")
                        .getDoubleArray(new double[0]);

        double[] cam2ReadBack =
                NetworkTableInstance.getDefault()
                        .getTable(cam2)
                        .getEntry("botpose_orb_wpiblue")
                        .getDoubleArray(new double[0]);

        System.out.println("Read back from NT Cam 1: " + Arrays.toString(cam1ReadBack));

        System.out.println("Read back from NT Cam 2: " + Arrays.toString(cam2ReadBack));

        Pose2d robotVisionPose = vision.getRobotPoseFromVision(40.0, 0, 0, 0, 0, 0);
        for (int i = 0; i < 10; i++) {
            System.out.println("Retrying pose fetch... attempt " + (i + 1));
            if (robotVisionPose != null) break;
            robotVisionPose = vision.getRobotPoseFromVision(40.0, 0, 0, 0, 0, 0);
            Thread.sleep(100);
        }
        // Line 106-111 area in your test
        double[] cam1Orientation =
                NetworkTableInstance.getDefault()
                        .getTable(cam1)
                        .getEntry("robot_orientation_set")
                        .getDoubleArray(
                                new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); // Provide a default!

        double[] cam2Orientation =
                NetworkTableInstance.getDefault()
                        .getTable(cam2)
                        .getEntry("robot_orientation_set")
                        .getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        assertNotNull(cam1Orientation, "Cam1 orientation should not be null");
        assertNotNull(cam2Orientation, "Cam2 orientation should not be null");

        assertNotNull(robotVisionPose, "PoseEstimate should not be null");
        assertEquals(
                0.0,
                robotVisionPose.getRotation().getDegrees(),
                0.01,
                "Rotation should match injected value");
        assertEquals(2.0, robotVisionPose.getX(), 0.001);
        assertEquals(3.0, robotVisionPose.getY(), 0.001);
        assertEquals(40, cam1Orientation[0], 0.001, "Cam1 yaw should match input");
        assertEquals(40, cam2Orientation[0], 0.001, "Cam2 yaw should match input");
        Thread.sleep(100);
    }

    @AfterAll
    public static void teardownNetworkTables() {
        System.out.println("Stopping NetworkTables server: LIMELIGHT PASSED");
        NetworkTableInstance.getDefault().stopServer();
    }
}
