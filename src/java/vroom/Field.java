package vroom;

import java.util.ArrayList;
import vroom.Obstacles.CircleObstacle;
import vroom.Obstacles.Obstacle;
import vroom.Obstacles.StuddedRectangle;
import vroom.Obstacles.WallObstacle;

public class Field {
    public enum obstacleType {
        CIRCLE,
        RECTANGLE,
        STUDDEDRECT,
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
            double x, double y, double strength, double width, double height, obstacleType type) {}

    public ArrayList<obstacle> obstacles = new ArrayList<>();

    public ArrayList<Obstacle> obstacleObjects = new ArrayList<>();

    public void setUpObstacles() {
        // walls
        obstacles.add(new obstacle(0, 0, .3, 16.540988, 0.0, obstacleType.WALL)); // bottom
        obstacles.add(new obstacle(16.540988, 0, .3, 0.0, 8.069326, obstacleType.WALL)); // right
        obstacles.add(new obstacle(0, 16.540988, .3, 16.540988, 0.0, obstacleType.WALL)); // top
        obstacles.add(new obstacle(0.0, 0.0, .3, 0.0, 8.069326, obstacleType.WALL)); // left

        // bump
        obstacles.add(
                new obstacle(
                        4.625594,
                        4.034663,
                        .2,
                        0.6477,
                        2.5411,
                        obstacleType.STUDDEDRECT)); // bump blue
        obstacles.add(
                new obstacle(
                        11.915394,
                        4.034663,
                        .2,
                        0.6477,
                        2.4511,
                        obstacleType.STUDDEDRECT)); // bump red
    }

    public Field() {
        setUpObstacles();
        initializeObstacleObjects();
    }

    public void initializeObstacleObjects() {
        for (obstacle obs : obstacles) {
            switch (obs.type()) {
                case CIRCLE:
                    obstacleObjects.add(
                            new CircleObstacle(obs.x(), obs.y(), obs.width(), obs.strength()));
                    break;
                case RECTANGLE:
                    obstacleObjects.add(
                            new vroom.Obstacles.Rectangle(
                                    obs.x(), obs.y(), obs.width(), obs.height(), obs.strength()));
                    break;
                case STUDDEDRECT:
                    obstacleObjects.add(
                            new StuddedRectangle(
                                    obs.x(),
                                    obs.y(),
                                    obs.width(),
                                    obs.height(),
                                    obs.strength(),
                                    1.5));
                    break;
                case WALL:
                    obstacleObjects.add(
                            new WallObstacle(
                                    obs.x(), obs.y(), obs.width(), obs.height(), obs.strength()));
                    break;
            }
        }
    }

    public ArrayList<Obstacle> getObstacles() {
        return obstacleObjects;
    }
}
