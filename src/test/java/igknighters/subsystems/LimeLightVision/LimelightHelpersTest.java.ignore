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
        double rot1 = 1;
        double rot2 = 359;
        LimeLightVisionReal vision = new LimeLightVisionReal(cam1, cam2);

        // Fake botpose array: [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
        double[] fakeBotpose1 = new double[18];
        fakeBotpose1[0] = 2.0; // x
        fakeBotpose1[1] = 3.0; // y
        fakeBotpose1[2] = 0.0; // z
        fakeBotpose1[3] = 0.0; // roll
        fakeBotpose1[4] = 0.0; // pitch
        fakeBotpose1[5] = rot1; // yaw in degrees
        fakeBotpose1[6] = 20.0; // latency
        fakeBotpose1[7] = 1; // tagCount
        fakeBotpose1[8] = 0.5; // tagSpan
        fakeBotpose1[9] = 1.0; // avgTagDist
        fakeBotpose1[10] = 0.1; // avgTagArea
        fakeBotpose1[11] = 1; // id
        fakeBotpose1[12] = 0; // txnc
        fakeBotpose1[13] = 0; // tync
        fakeBotpose1[14] = 0; // ta
        fakeBotpose1[15] = 0; // distToCamera
        fakeBotpose1[16] = 0; // distToRobot
        fakeBotpose1[17] = 0; // ambiguity

        // [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
        double[] fakeBotpose2 = new double[18];
        fakeBotpose2[0] = 2.0; // x
        fakeBotpose2[1] = 3.0; // y
        fakeBotpose2[2] = 0.0; // z
        fakeBotpose2[3] = 0.0; // roll
        fakeBotpose2[4] = 0.0; // pitch
        fakeBotpose2[5] = rot2; // yaw in degrees
        fakeBotpose2[6] = 20.0; // latency
        fakeBotpose2[7] = 1; // tagCount
        fakeBotpose2[8] = 0.5; // tagSpan
        fakeBotpose2[9] = 1.0; // avgTagDist
        fakeBotpose2[10] = 0.1; // avgTag
        fakeBotpose2[11] = 2; // id
        fakeBotpose2[12] = 0; // txnc
        fakeBotpose2[13] = 0; // tync
        fakeBotpose2[14] = 0; // ta
        fakeBotpose2[15] = 0; // distToCamera
        fakeBotpose2[16] = 0; // distToRobot
        fakeBotpose2[17] = 0; // ambiguity
        // Inject into NetworkTables
        NetworkTableInstance.getDefault()
                .getTable(cam1)
                .getEntry("botpose_orb_wpiblue")
                .setDoubleArray(fakeBotpose1);

        NetworkTableInstance.getDefault()
                .getTable(cam1)
                .getEntry("botpose_wpiblue")
                .setDoubleArray(fakeBotpose1);
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
        double[] cam1Orientation =
                NetworkTableInstance.getDefault()
                        .getTable(cam1)
                        .getEntry("robot_orientation_set")
                        .getDoubleArray(new double[0]);
        double[] cam2Orientation =
                NetworkTableInstance.getDefault()
                        .getTable(cam2)
                        .getEntry("robot_orientation_set")
                        .getDoubleArray(new double[0]);

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
