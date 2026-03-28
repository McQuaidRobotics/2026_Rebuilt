// package igknighters.util;

// import edu.wpi.first.math.geometry.Pose2d;
// import igknighters.Robot;
// import igknighters.util.log.Log;

// public class RobotPosePredError {
//     Pose2d[] predictionHistory = new Pose2d[2];
//     int writeIndex = 0;
//     public void logPose(Pose2d currentPose)
//     {
//         predictionHistory[writeIndex] = Robot.pose_pred.getPredictedPose(currentPose);
//         if (writeIndex != 1) {
//             writeIndex++;
//         }
//         else {
//             writeIndex = 0;
//         }
//     }

//     Log.Log(predictionHistory[0]-predictionHistory[1]);

//     public double findError() {

//     }
// }
