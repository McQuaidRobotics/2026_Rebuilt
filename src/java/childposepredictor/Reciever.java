package childposepredictor;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Reciever {
    NetworkTableInstance tableInstance;

    NetworkTable output = NetworkTableInstance.getDefault().getTable("CHILD_POSE_PREDICTOR");
    public Reciever() {
        tableInstance = NetworkTableInstance.getDefault();
    }

    public double[] getOutput() {
        return output.getEntry("future_pos").getDoubleArray(new double[]{0.0, 0.0, 0.0});
    }

    public boolean hasTarget() {
        return output.getEntry("HAS_TARGET").getBoolean(false);
    }
}
