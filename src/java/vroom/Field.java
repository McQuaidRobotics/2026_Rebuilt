package vroom;

import java.util.ArrayList;
import vroom.Obstacles.*;

public interface Field {
    public enum obstacleType {
        CIRCLE,
        RECTANGLE,
        STUDDED_RECTANGLE,
        FUNNEL,
        RECTANGLENOTADJACENT,
        CORIDOR,
        WALL;
    }

    /**
     * {@summary}Holds the data for an obstacle used in the repulsor field navigation system.
     *
     * @param obstaclePose The Pose2d representing the position of the obstacle rotation is ignored
     *     measured from center of obstacle.
     * @param strength The repulsion strength of the obstacle. 1 is a good starting point.
     * @param width The width of the obstacle (center to edge)(used for visualization or collision
     *     detection) this could will be the radius for CIRCLE types. on a wall height =0 means
     *     horizontal
     * @param height The height of the obstacle (center to edge)(used for visualization or collision
     *     detection).
     */
    public record obstacle(
            double x,
            double y,
            double strength,
            double width,
            double height,
            boolean horizontal,
            obstacleType type) {}

    ArrayList<Obstacle> getObstacles();
}
