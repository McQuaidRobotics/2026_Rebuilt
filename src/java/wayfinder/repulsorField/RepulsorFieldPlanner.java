package wayfinder.repulsorField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import wayfinder.controllers.CircularSlewRateLimiter;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;

public class RepulsorFieldPlanner {
    private final PositionalController controller;
    private final CircularSlewRateLimiter rotationRateLimiter =
            new CircularSlewRateLimiter(Math.PI * 5.0);
    private final List<Obstacle> fixedObstacles = new ArrayList<>();

    // Removed the global netForceVec since we will instantiate it locally

    public RepulsorFieldPlanner(PositionalController controller, Obstacle... obstacles) {
        fixedObstacles.addAll(List.of(obstacles));
        this.controller = controller;
    }

    // Consolidated into a single getForce method returning a Translation2d
    Translation2d getForce(Translation2d curLocation, Translation2d goal) {
        // push towards goal
        // System.out.println("CURRENT POSE: " + curLocation.getX() + ", " + curLocation.getY());
        double xForceGoal = 0.0;
        double yForceGoal = 0.0;
        double currentX = curLocation.getX(); // to ensure nothing is being mutated somehow
        double currentY = curLocation.getY();
        double xDisplacement = goal.getX() - curLocation.getX();
        double yDisplacement = goal.getY() - curLocation.getY();
        double norm = Math.hypot(xDisplacement, yDisplacement);
        if (norm != 0) {
            double cos = xDisplacement / norm;
            double sin = yDisplacement / norm;
            double mag = (1 + 1.0 / (1e-6 + norm));
            xForceGoal = mag * cos * 2.0;
            yForceGoal = mag * sin * 2.0;
        }

        // push away from obstacles
        double xForceObs = 0.0;
        double yForceObs = 0.0;
        for (Obstacle obs : fixedObstacles) {
            // System.out.println(
            // "IN LOOP POSE PASSED IN: " + curLocation.getX() + ", " + curLocation.getY());
            Translation2d force = obs.getForceAtPosition(curLocation, goal);
            if (!Double.isFinite(xForceObs) || !Double.isFinite(yForceObs)) {
                continue;
            }
            xForceObs += force.getX();
            if (force.getY() >= 1000) {
                // System.out.println("Invalid force detected!");
            }
            yForceObs += force.getY();
            if (curLocation.getX() != currentX || curLocation.getY() != currentY) {
                // System.out.println("MUTATION DETECTED WOWZERS HOW THE HELL DID YOU MANAGE
                // THAT?");
            }
        }

        // Return a fresh, standard Translation2d
        return new Translation2d(xForceGoal + xForceObs, yForceGoal + yForceObs);
    }

    public FieldSpeeds calculate(
            double period,
            Pose2d measurement,
            Speeds measurementVelo,
            Pose2d target,
            ChassisConstraints constraints) {
        double straightDist = measurement.getTranslation().getDistance(target.getTranslation());
        Pose2d intermediatePose = target;

        if (straightDist > 0.375) {
            // Capture the returned Translation2d
            Translation2d netForceVec =
                    getForce(measurement.getTranslation(), target.getTranslation());

            // Use the standard immutable .times() method
            netForceVec = netForceVec.times(straightDist / netForceVec.getNorm());

            Rotation2d targetDirection = netForceVec.getAngle();
            Rotation2d limited =
                    new Rotation2d(rotationRateLimiter.calculate(targetDirection.getRadians()));

            Translation2d rotatedNetForceVec = netForceVec.rotateBy(limited.minus(targetDirection));

            Translation2d goalPosition = measurement.getTranslation().plus(rotatedNetForceVec);
            intermediatePose = new Pose2d(goalPosition, target.getRotation());
        }

        Logger.recordOutput("Pose/Wayfinder/IntermediatePose", intermediatePose);
        return controller.calculate(
                period,
                measurement,
                measurementVelo.asFieldRelative(measurement.getRotation()),
                intermediatePose,
                constraints);
    }

    public void reset(Pose2d measurement, FieldSpeeds measurementVelo, Pose2d target) {
        controller.reset(measurement, measurementVelo, target);
        rotationRateLimiter.reset(
                target.getTranslation()
                        .minus(measurement.getTranslation())
                        .getAngle()
                        .getRadians());
    }

    public Pose2d[] getArrows(Translation2d goal, double xCount, double yCount) {
        final double FIELD_WIDTH = FieldConstants.Y_FIELD;
        final double FIELD_LENGTH = FieldConstants.X_FIELD;
        Pose2d[] arrows = new Pose2d[(int) (xCount * yCount + yCount + 1)];
        for (int x = 0; x <= xCount; x++) {
            for (int y = 0; y <= yCount; y++) {
                Translation2d translation =
                        new Translation2d(x * (FIELD_LENGTH) / xCount, y * FIELD_WIDTH / yCount);
                Translation2d force = getForce(translation, goal);
                Rotation2d rotation;
                if (force.getNorm() > 1e-6) {
                    rotation = force.getAngle();
                } else {
                    rotation = Rotation2d.kZero;
                }
                arrows[x * (int) yCount + y] = new Pose2d(translation, rotation);
            }
        }

        return arrows;
    }
}
