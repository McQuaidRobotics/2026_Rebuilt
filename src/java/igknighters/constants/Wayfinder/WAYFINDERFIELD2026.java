package igknighters.constants.Wayfinder;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.FieldConstants;
import wayfinder.repulsorField.Obstacle;

public class WAYFINDERFIELD2026 {

    private static final Translation2d[] BUMP_VERTICIES =
            new Translation2d[] {
                new Translation2d(
                        FieldConstants.BUMP.BUMP_1_X_METERS - FieldConstants.BUMP.HALF_X_METERS,
                        FieldConstants.BUMP.BUMP_1_Y_METERS
                                - FieldConstants.BUMP.HALF_Y_METERS), // Bottom-left vertex BUMP 1
                new Translation2d(
                        FieldConstants.BUMP.BUMP_2_X_METERS - FieldConstants.BUMP.HALF_X_METERS,
                        FieldConstants.BUMP.BUMP_2_Y_METERS
                                - FieldConstants.BUMP.HALF_Y_METERS), // Bottom-left vertex BUMP 2
            };

    private static final Obstacle[] BUMP_OBSTACLES_BLUE;

    static {
        BUMP_OBSTACLES_BLUE = new Obstacle[BUMP_VERTICIES.length];
        for (int i = 0; i < BUMP_VERTICIES.length; i++) {
            BUMP_OBSTACLES_BLUE[i] =
                    new Obstacle.RectangleObstacle(
                            BUMP_VERTICIES[i],
                            new Translation2d(
                                    FieldConstants.BUMP.HALF_X_METERS * 2,
                                    FieldConstants.BUMP.HALF_Y_METERS * 2),
                            .30,
                            .5);
        }
    }

    private static final Obstacle[] WALL_OBSTACLES = {
        new Obstacle.HorizontalObstacle(0, .3, .75, true),
        new Obstacle.HorizontalObstacle(FieldConstants.X_FIELD, .3, .75, false),
        new Obstacle.VerticalObstacle(0, 0, .75, true),
        new Obstacle.VerticalObstacle(FieldConstants.Y_FIELD, 0, .75, false)
    };

    public static final Obstacle[] ALL_OBSTACLES =
            new Obstacle[WALL_OBSTACLES.length + BUMP_OBSTACLES_BLUE.length];

    static {
        System.arraycopy(WALL_OBSTACLES, 0, ALL_OBSTACLES, 0, WALL_OBSTACLES.length);
        System.arraycopy(
                BUMP_OBSTACLES_BLUE,
                0,
                ALL_OBSTACLES,
                WALL_OBSTACLES.length,
                BUMP_OBSTACLES_BLUE.length);
    }
}
