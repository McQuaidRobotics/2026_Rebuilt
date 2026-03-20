package igknighters.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.commands.Repulsor.obstacle;
import igknighters.constants.FieldConstants;

public class Wayfinder {
    
        ArrayList<obstacle> obstacles = FieldConstants.OBSTACLES.ALL_WAYFINDER_OBSTACLES;
    
    public Pose2d[] wayfind(Pose2d currentPose, Pose2d goalPose)
    {
        Pose2d[] path = new Pose2d[1];
        

        //the rotational step distance when searching for path (degrees)
        double omegaStep = 15;

        //the translational step distance when searching for path (meters)
        double translationalStep = .1;
        
        do {
            path[0] = currentPose;
            score(path[-1]);

        } while (!goalState(path[-1], goalPose));


        return path;
    }

    //check if at goal state with 0.5 meter tolerance
    public boolean goalState(Pose2d lastPathPose, Pose2d goalPose)
    {
        double tolerance = 0.5;
        if (Math.abs(goalPose.getX() - lastPathPose.getX()) < tolerance && Math.abs(goalPose.getY()-lastPathPose.getY()) < tolerance)
        {
            return true;
        }
        return false;
    }

    public double score(Pose2d potentialPose)
    {
            for (obstacle obs:obstacles)
            {

            }
    }
    
}
