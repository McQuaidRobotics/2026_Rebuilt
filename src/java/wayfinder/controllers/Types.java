package wayfinder.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import wpilibExt.Speeds.FieldSpeeds;

public class Types {
    public record State(double position, double velocity) {
        public static final State kZero = new State(0, 0);
    }

    public interface Controller<Measurement, MeasurementRate, Target, Constraints> {
        MeasurementRate calculate(
                double period,
                Measurement measurement,
                MeasurementRate measurementRate,
                Target target,
                Constraints constraints);

        void reset(Measurement measurement, MeasurementRate measurementRate, Target target);

        boolean isDone(Measurement measurement, Target target);
    }

    public record Constraints(double maxVelocity, double maxAcceleration) {}

    public record ChassisConstraints(Constraints translation, Constraints rotation) {
        public static final ChassisConstraints kZero =
                new ChassisConstraints(new Constraints(0, 0), new Constraints(0, 0));
    }

    public interface PathFollower {
        FieldSpeeds calculate(double dt, Pose2d currentPose, Pose2d targetPose);

        boolean isDone(Pose2d currentPose, Pose2d targetPose);

        void reset(Pose2d currentPose, Pose2d targetPose);
    }
}
