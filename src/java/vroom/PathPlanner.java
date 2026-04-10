package vroom;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PathPlanner {
    private final Field field;
    private final double maxVelocity; // meters per second
    private final double lookaheadDistance = 0.1; // step size in meters
    private final double obstacleInfluenceRange = 1.5; // distance where obstacles start pushing
    static PIDController xController = new PIDController(0.1, 0, 0);
    static PIDController yController = new PIDController(0.1, 0, 0);
    static PIDController thetaController = new PIDController(1.0, 0, 0);

    public PathPlanner(Field field, double maxVelocity) {
        this.field = field;
        this.maxVelocity = maxVelocity;
    }

    public record PathPoint(Pose2d pose, double timeSeconds, double velocity) {}

    public PathPoint[] generateTimestampedPath(Pose2d start, Pose2d target) {
        List<PathPoint> path = new ArrayList<>();
        Pose2d cursor = start;
        double currentTime = 0;
        double maxVel = 3.0; // Meters per second

        ArrayList<Pose2d> displayPose = new ArrayList<>();

        path.add(new PathPoint(cursor, 0, maxVel));

        int maxSteps = 500;
        while (cursor.getTranslation().getDistance(target.getTranslation()) > 0.1
                && path.size() < maxSteps) {
            Translation2d force =
                    calculateTotalForce(cursor.getTranslation(), target.getTranslation());
            Rotation2d moveDir = force.getAngle();

            // 0.1m step
            Translation2d nextStep = cursor.getTranslation().plus(new Translation2d(0.1, moveDir));

            // --- TIMING LOGIC ---
            // Time = Distance / Velocity
            double stepTime = 0.1 / maxVel;
            currentTime += stepTime;

            if (path.size() % 10 == 0 || path.size() == maxSteps || path.size() == 1) {
                displayPose.add(cursor);
            }

            cursor = new Pose2d(nextStep, moveDir);
            path.add(new PathPoint(cursor, currentTime, maxVel));
        }

        Logger.recordOutput("PATH_DISPLAY", displayPose.toArray(new Pose2d[0]));
        return path.toArray(new PathPoint[0]);
    }

    public Pose2d[] generatePath(Pose2d current, Pose2d target) {
        Pose2d cursor = current;
        int maxSteps = 500;
        // Use an ArrayList temporarily so we don't have to deal with nulls
        java.util.ArrayList<Pose2d> pathList = new java.util.ArrayList<>();
        ArrayList<Pose2d> posesToDisplay = new ArrayList<>();
        System.out.println(
                "DISTANCE: " + cursor.getTranslation().getDistance(target.getTranslation()));
        while (cursor.getTranslation().getDistance(target.getTranslation()) > 0.1
                && pathList.size() < maxSteps) {
            Translation2d force =
                    calculateTotalForce(cursor.getTranslation(), target.getTranslation());

            // Move cursor in direction of force by lookahead distance
            Translation2d nextStep =
                    cursor.getTranslation()
                            .plus(new Translation2d(lookaheadDistance, force.getAngle()));

            cursor = new Pose2d(nextStep, target.getRotation());
            pathList.add(cursor);
            if (pathList.size() % 2 == 0 || pathList.size() == maxSteps || pathList.size() == 1) {
                posesToDisplay.add(cursor);
            }
        }

        // Convert to a clean array with NO null values
        Pose2d[] pathArray = pathList.toArray(new Pose2d[0]);

        Pose2d[] posesToDisplayArray = posesToDisplay.toArray(new Pose2d[0]);

        // Log the clean array
        // Logger.recordOutput("PATH", pathArray);
        Logger.recordOutput("PATH_DISPLAY", posesToDisplayArray);

        return pathArray;
    }

    private Translation2d calculateTotalForce(Translation2d current, Translation2d target) {
        Translation2d attractive = target.minus(current);
        double distToTarget = attractive.getNorm();
        Translation2d unitAttractive =
                (distToTarget > 0) ? attractive.div(distToTarget) : new Translation2d();

        // Higher attraction weight helps "pull" the robot through narrow gaps
        Translation2d finalAttractive = unitAttractive.times(5);

        Translation2d totalRepulsive = new Translation2d();
        double robotRadius = 0.5;

        ArrayList<vroom.Obstacles.Obstacle> corridors = new ArrayList<>();

        for (vroom.Obstacles.Obstacle obs : field.getObstacles()) {
            if (obs instanceof vroom.Obstacles.CORIDOR) {
                corridors.add(obs);
                continue;
            }
            totalRepulsive =
                    totalRepulsive.plus(
                            obs.calculateForce(current, target, robotRadius, totalRepulsive));
        }
        Translation2d totalForce = finalAttractive.plus(totalRepulsive);
        for (vroom.Obstacles.Obstacle obs : corridors) {
            totalForce =
                    totalForce.plus(obs.calculateForce(current, target, robotRadius, totalForce));
        }
        return totalForce;
    }

    public Pose2d getInterpolatedPose(PathPoint[] path, double time) {
        if (time <= 0) return path[0].pose();
        if (time >= path[path.length - 1].timeSeconds()) return path[path.length - 1].pose();

        // 1. Find the two points we are between
        for (int i = 0; i < path.length - 1; i++) {
            if (time < path[i + 1].timeSeconds()) {
                PathPoint start = path[i];
                PathPoint end = path[i + 1];

                // 2. Calculate % of completion between these two points
                double t = (time - start.timeSeconds()) / (end.timeSeconds() - start.timeSeconds());

                // 3. Interpolate Translation and Rotation
                Translation2d lerpTrans =
                        start.pose().getTranslation().interpolate(end.pose().getTranslation(), t);
                Rotation2d lerpRot =
                        start.pose().getRotation().interpolate(end.pose().getRotation(), t);

                return new Pose2d(lerpTrans, lerpRot);
            }
        }
        return path[path.length - 1].pose();
    }

    public ChassisSpeeds calculateSpeeds(Pose2d currentPose, PathPoint[] path, double time) {
        // Get where we should be 0.1s in the future (Lookahead)
        Pose2d setpoint = getInterpolatedPose(path, time + 0.1);

        // PID helps fix errors
        double xFeedback = xController.calculate(currentPose.getX(), setpoint.getX());
        double yFeedback = yController.calculate(currentPose.getY(), setpoint.getY());

        // Feedforward: "The path is moving this fast, so start with this speed"
        // (Velocity from our PathPoint)
        double velocity = 3.0;
        Rotation2d direction =
                setpoint.getTranslation().minus(currentPose.getTranslation()).getAngle();

        double xFF = direction.getCos() * velocity;
        double yFF = direction.getSin() * velocity;

        return new ChassisSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        setpoint.getRotation().getRadians()));
    }

    public Pose2d getLookaheadPose(Pose2d currentPose, Pose2d[] path, double lookaheadDist) {
        if (path.length == 0) return currentPose;

        // 1. Find the index of the point on the path closest to the robot
        int closestIndex = 0;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < path.length; i++) {
            double dist = currentPose.getTranslation().getDistance(path[i].getTranslation());
            if (dist < minDistance) {
                minDistance = dist;
                closestIndex = i;
            }
        }

        // 2. Calculate how many steps to look ahead
        // Since each step in our path is 'lookaheadDistance' (0.1m) apart:
        int stepsAhead = (int) Math.round(lookaheadDist / this.lookaheadDistance);
        int targetIndex = Math.min(closestIndex + stepsAhead, path.length - 1);

        return path[targetIndex];
    }
}
