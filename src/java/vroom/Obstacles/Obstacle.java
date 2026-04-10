package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public interface Obstacle {
    /**
     * Calculates the repulsive force vector this obstacle exerts on the robot.
     *
     * @param robotPos Current position of the pathfinding cursor.
     * @param targetPos The final goal (needed for "Sliding" logic).
     * @param robotRadius The physical size of the robot.
     * @return Translation2d representing the force vector.
     */
    Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar);
}
