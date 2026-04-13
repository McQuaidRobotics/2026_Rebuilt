package vroom.Fields;

import java.util.ArrayList;
import vroom.Field;
import vroom.Obstacles.BUMPOBSTACLE;
import vroom.Obstacles.CORIDOR;
import vroom.Obstacles.CircleObstacle;
import vroom.Obstacles.FunnelObstacle;
import vroom.Obstacles.Obstacle;
import vroom.Obstacles.StuddedRectangle;
import vroom.Obstacles.WallObstacle;

public class REBUILTBUMP implements Field {
    public ArrayList<obstacle> obstacles = new ArrayList<>();

    public ArrayList<Obstacle> obstacleObjects = new ArrayList<>();

    public void setUpObstacles() {
        // walls
        obstacles.add(new obstacle(0, 0, .3, 16.540988, 0.0, false, obstacleType.WALL)); // bottom
        obstacles.add(
                new obstacle(16.540988, 0, .3, 0.0, 8.069326, false, obstacleType.WALL)); // right
        obstacles.add(
                new obstacle(0, 16.540988, .3, 16.540988, 0.0, false, obstacleType.WALL)); // top
        obstacles.add(new obstacle(0.0, 0.0, .3, 0.0, 8.069326, false, obstacleType.WALL)); // left

        // HUB
        obstacles.add(
                new obstacle(
                        4.625594,
                        4.034663,
                        1.0,
                        0.6477,
                        2.5411,
                        false,
                        obstacleType.CIRCLE)); // bump blue
        obstacles.add(
                new obstacle(
                        11.915394,
                        4.034663,
                        1.0,
                        0.6477,
                        2.4511,
                        false,
                        obstacleType.CIRCLE)); // bump red

        // TRENCHES
        obstacles.add(new obstacle(11.915394, 6.45 + 2, 1.0, 2.0, 1.0, true, obstacleType.CIRCLE));

        // red top

        obstacles.add(
                new obstacle(11.915394, 1.46 - 2.0, 1.0, 2.0, 1.0, true, obstacleType.CIRCLE));

        // red bottom

        obstacles.add(new obstacle(4.625594, 6.45 + 2.0, 1.0, 2.0, 1.0, true, obstacleType.CIRCLE));

        // blue top

        obstacles.add(new obstacle(4.625594, 1.46 - 2.0, 1.0, 2.0, 1.0, true, obstacleType.CIRCLE));
        // blue bottom

        // CLIMB BELOW

        obstacles.add(new obstacle(0.0, 4.323588, 1.0, 2.5, 0, false, obstacleType.CIRCLE));
        obstacles.add(new obstacle(16.540988, 4.323588, 1.0, 2.5, 0, false, obstacleType.CIRCLE));
    }

    public REBUILTBUMP() {
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

                case CORIDOR:
                    obstacleObjects.add(
                            new CORIDOR(
                                    obs.x(),
                                    obs.y(),
                                    obs.width(),
                                    obs.height(),
                                    obs.strength(),
                                    obs.horizontal()));
                    break;
                case RECTANGLENOTADJACENT:
                    obstacleObjects.add(
                            new BUMPOBSTACLE(
                                    obs.x(), obs.y(), obs.width(), obs.height(), obs.strength()));
                    break;
                case FUNNEL:
                    obstacleObjects.add(
                            new FunnelObstacle(
                                    obs.x(),
                                    obs.y(),
                                    obs.width(),
                                    obs.height(),
                                    2.0,
                                    obs.strength(),
                                    1.5));
                    break;
                case STUDDED_RECTANGLE:
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
