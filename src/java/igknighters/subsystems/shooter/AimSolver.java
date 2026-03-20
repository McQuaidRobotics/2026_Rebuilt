package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.FieldVisualizer;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.log.Log;
import org.littletonrobotics.junction.Logger;

public class AimSolver {

    public static class LERP_SOLVERS {
        static enum SHOT_TYPE {
            HUB,
            PASS
        }

        // distance to RPM mapping
        static LerpTable hubRPMTable =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 2800.0),
                            new LerpTableEntry(3.0, 3000.0),
                            new LerpTableEntry(5.0, 4000.0),
                            new LerpTableEntry(10.0, 4500.0),
                            new LerpTableEntry(15.0, 5500.0),
                            new LerpTableEntry(20.0, 6000.0),
                        });
        // distance to hood angle mapping
        static LerpTable hubHoodTable =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 10.0),
                            new LerpTableEntry(3.0, 20.0),
                            new LerpTableEntry(5.0, 30.0),
                            new LerpTableEntry(10.0, 40.0),
                            new LerpTableEntry(15.0, 50.0),
                            new LerpTableEntry(20.0, 60.0),
                        });

        static LerpTable hubTofTable =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 1.0), new LerpTableEntry(2.0, 2.0)
                        });

        static LerpTable passRPMTable =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 2800.0),
                            new LerpTableEntry(3.0, 3000.0),
                            new LerpTableEntry(5.0, 4000.0),
                            new LerpTableEntry(10.0, 4500.0),
                            new LerpTableEntry(15.0, 5500.0),
                            new LerpTableEntry(20.0, 6000.0),
                        });
        // distance to hood angle mapping
        static LerpTable passHoodTable =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 10.0),
                            new LerpTableEntry(3.0, 20.0),
                            new LerpTableEntry(5.0, 30.0),
                            new LerpTableEntry(10.0, 40.0),
                            new LerpTableEntry(15.0, 50.0),
                            new LerpTableEntry(20.0, 60.0),
                        });

        static LerpTable passTofTable =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 1.0), new LerpTableEntry(2.0, 2.0)
                        });

        public static ShooterState solve(
                Pose3d shooterPose, Pose3d targetPose, ChassisSpeeds speeds, SHOT_TYPE shotType) {
            double sx = shooterPose.getX();
            double sy = shooterPose.getY();

            double tx = targetPose.getX();
            double ty = targetPose.getY();

            // CRITICAL: These MUST be field-relative speeds, not robot-relative!
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;

            // 1. Get initial distance
            double initialDistance = Math.hypot(tx - sx, ty - sy);

            double px = tx;
            double py = ty;
            double predictedDistance = initialDistance;

            // 2. Iterate to find the true Virtual Target
            // t changes based on distance but at some point it will start to become 0
            for (int i = 0; i < 3; i++) {
                double t;
                if (shotType == SHOT_TYPE.HUB) {
                    t = hubTofTable.lerp(predictedDistance);
                } else {
                    t = passTofTable.lerp(predictedDistance);
                }

                // SUBTRACT velocity to shift the target in the opposite direction of movement
                px = tx - (vx * t);
                py = ty - (vy * t);

                // Recalculate distance to the new virtual target
                predictedDistance = Math.hypot(px - sx, py - sy);
            }

            // 3. Find delta to Virtual Target (Destination - Start)
            double dx = px - sx;
            double dy = py - sy;

            // 4. Calculate Absolute Angle using atan2
            double absoluteFieldAngle = Math.atan2(dy, dx);
            double robotTheta = shooterPose.getRotation().getZ();

            // Wrap the angle safely using atan2(sin, cos) so the turret takes the shortest path
            double turretTheta =
                    Math.atan2(
                            Math.sin(absoluteFieldAngle - robotTheta),
                            Math.cos(absoluteFieldAngle - robotTheta));

            // 5. Get hardware setpoints using the fully-calculated Virtual Distance
            double predictedRPM;
            double predictedHoodAngle;
            if (shotType == SHOT_TYPE.HUB) {
                predictedRPM = hubRPMTable.lerp(predictedDistance);
                predictedHoodAngle = hubHoodTable.lerp(predictedDistance);
            } else {
                predictedRPM = passRPMTable.lerp(predictedDistance);
                predictedHoodAngle = passHoodTable.lerp(predictedDistance);
            }

            return new ShooterState(
                    RPM.of(predictedRPM), Radians.of(-turretTheta), Degrees.of(predictedHoodAngle));
        }
    }

    public static class Solvers {

        static final double EfficiencyConst = 2.1;
        static LerpTable maxHeightDistanceLerp =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1, 2.5),
                            new LerpTableEntry(2, 3.0),
                            new LerpTableEntry(3, 3.2),
                            new LerpTableEntry(4, 3.4),
                            new LerpTableEntry(5, 3.6),
                            new LerpTableEntry(6, 3.9),
                            new LerpTableEntry(10, 5.0)
                        });
        static LerpTable airResistanceDTodistancedivider =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 2.4),
                            new LerpTableEntry(3.0, 2.0),
                            new LerpTableEntry(3.6, 1.8),
                            new LerpTableEntry(4.0, 1.9),
                            new LerpTableEntry(5.0, 2.0),
                            new LerpTableEntry(6, 2.15),
                            new LerpTableEntry(10.0, 1.65),
                            new LerpTableEntry(15.0, 1.5),
                            new LerpTableEntry(20.0, 1.485),
                        });

        static Mechanism2d canSHOOTMECH = new Mechanism2d(20, 20);
        static boolean hasBeenAdded = false;

        public static void publishOnce() {
            if (!hasBeenAdded) {
                SmartDashboard.putData("Visualizers/Shooter/CAN SHOOT", canSHOOTMECH);
            }
            hasBeenAdded = true;
        }

        public static TunableDouble effiencyConst =
                TunableValues.getDouble("Shooter/EfficiencyConst", 2.0);

        public static void canShoot(boolean canShoot) {
            publishOnce();
            if (canShoot) {
                canSHOOTMECH.setBackgroundColor(new Color8Bit(Color.kGreen));
            } else {
                canSHOOTMECH.setBackgroundColor(new Color8Bit(Color.kRed));
            }
        }

        public static ShooterState solve_max_height_iterative(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double periodTime) {
            // 1. Position and Target setup

            Pose2d predictedPose = Robot.pose_pred.getPredictedPose(shooterPose.toPose2d());
            double sx = predictedPose.getX();
            double sy = predictedPose.getY();
            double sz = shooterPose.getZ();

            double initialDist =
                    shooterPose.getTranslation().getDistance(targetPose.getTranslation());
            double estimatedToF = 1.5;
            // if (initialDist <= 8) {
            //     estimatedToF = initialDist / 1.5; // Assume 5m/s avg horizontal velocity
            // } else {
            //     estimatedToF = 1; // Assume 5m/s avg horizontal velocity
            // }
            // 3. TARGET PROJECTION: Scale the target lead by (ToF + Latency)
            // We subtract the robot's velocity because from the ball's perspective,
            // the target is moving toward/away at the robot's speed.
            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx =
                    targetPose.getX()
                            + airResistanceAdder.getX()
                            - (speeds.vxMetersPerSecond * (estimatedToF));
            double ty =
                    targetPose.getY()
                            + airResistanceAdder.getY()
                            - (speeds.vyMetersPerSecond * (estimatedToF));
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy); // Total horizontal distance

            double bestRPM = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestV = 0.0;
            double minRPMDiff = Double.MAX_VALUE;

            // 2. Iterative Arc Search
            for (int i = 0; i < 3; i++) {
                // Search heights between target + 1m and max ceiling
                double currentCeilingHeight = tz + 2.0 + (i * (maxHeightMeters - (tz + 2.0)) / 3.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                if (hRise < 0 || hFall < 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                double vx_planar = floorDistance / totalTime;
                double vz_initial = 9.81 * tRise; // Velocity needed to reach peak
                double v_total = Math.hypot(vx_planar, vz_initial);

                // Single-sided flywheel: Wheel surface speed = 2x Ball speed
                double RPM =
                        (v_total / (2 * Math.PI * kFlywheels.WHEEL_RADIUS_METERS))
                                * 60
                                * EfficiencyConst;

                double launchAngleDegrees = Math.toDegrees(Math.atan2(vz_initial, vx_planar));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) continue;

                // Choose the shot closest to our current flywheel speed for faster spin-up
                double dRPM = Math.abs(currentRPM - RPM);
                if (dRPM < minRPMDiff) {
                    minRPMDiff = dRPM;
                    bestRPM = RPM;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestV = v_total;
                }
            }

            // 3. Final Angles
            double absoluteFieldAngle = Math.atan2(dy, dx);
            double robotYawFuture = predictedPose.getRotation().getRadians();
            double turretAngle =
                    Math.atan2(
                            Math.sin(absoluteFieldAngle - robotYawFuture),
                            Math.cos(absoluteFieldAngle - robotYawFuture));

            if (bestRPM != 0 && bestThetaHoodDegrees != 0) {
                canShoot(true);
                // We pass absoluteFieldAngle so the trajectory line points at the target
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        absoluteFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            if (Robot.isBlue()) {
                return new ShooterState(
                        RPM.of(bestRPM),
                        Radians.of(-turretAngle),
                        Degrees.of(bestThetaHoodDegrees));
            } else {
                return new ShooterState(
                        RPM.of(bestRPM),
                        Radians.of(-turretAngle),
                        Degrees.of(bestThetaHoodDegrees));
            }
        }

        /**
         * MAKES THE PARABOLA FALL BACK TO THE MINIMUM HEIGHT
         *
         * @param shooterPose The pose of the shooter where the balls leave
         * @param targetPose The pose of the target
         * @param speeds The chassis speeds
         * @param currentRPM The current RPM of the shooter
         * @param maxHeightMeters The maximum height the ball can reach
         * @param minHeightMeters The minimum height the ball can reach
         * @param periodTime The loop time eg 20 ms
         * @return The calculated shooter state
         */
        public static ShooterState solve_max_and_min_iterative(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double minHeightMeters,
                double periodTime) {
            Pose2d predictedPose = Robot.pose_pred.getPredictedPose(shooterPose.toPose2d());
            FieldVisualizer.getInstance().updatePredictedPose(predictedPose);
            double sx = predictedPose.getX();
            double sy = predictedPose.getY();
            double sz = shooterPose.getZ();

            double initialDist =
                    shooterPose.getTranslation().getDistance(targetPose.getTranslation());
            Log.log("ROBOT/Commands/AimSolver/Distance", initialDist);
            double estimatedToF = 1.5;
            // if (initialDist <= 8) {
            //     estimatedToF = initialDist / 1.5; // Assume 5m/s avg horizontal velocity
            // } else {
            //     estimatedToF = 1; // Assume 5m/s avg horizontal velocity
            // }

            // 3. TARGET PROJECTION: Scale the target lead by (ToF + Latency)
            // We subtract the robot's velocity because from the ball's perspective,
            // the target is moving toward/away at the robot's speed.
            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx =
                    targetPose.getX()
                            + airResistanceAdder.getX()
                            - (speeds.vxMetersPerSecond * (estimatedToF));
            double ty =
                    targetPose.getY()
                            + airResistanceAdder.getY()
                            - (speeds.vyMetersPerSecond * (estimatedToF));
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy); // Total horizontal distance

            double bestRPM = 0.0;
            double bestThetaHoodDegrees = SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES;
            double bestV = 0.0;
            double highestArc = 0.0;

            // 2. Iterative Arc Search
            for (int i = 0; i < 5; i++) {
                // Search heights between target + 1m and max ceiling
                double currentCeilingHeight =
                        minHeightMeters + (i * (maxHeightMeters - minHeightMeters) / 5.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                if (hRise < 0 || hFall < 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                double vx_planar = floorDistance / totalTime;
                double vz_initial = 9.81 * tRise; // Velocity needed to reach peak
                double v_total = Math.hypot(vx_planar, vz_initial);

                // Single-sided flywheel: Wheel surface speed = 2x Ball speed
                double RPM =
                        (v_total / (2 * Math.PI * kFlywheels.WHEEL_RADIUS_METERS))
                                * 60
                                * EfficiencyConst;

                double launchAngleDegrees = Math.toDegrees(Math.atan2(vz_initial, vx_planar));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) continue;

                // Choose the shot with the highest arc
                double arc = launchAngleDegrees;
                if (arc > highestArc) {
                    highestArc = arc;
                    bestRPM = RPM;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestV = v_total;
                }
            }

            // 3. Final Angles
            double absoluteFieldAngle = Math.atan2(dy, dx);
            double robotYawFuture = predictedPose.getRotation().getRadians();
            double turretAngle =
                    Math.atan2(
                            Math.sin(absoluteFieldAngle - robotYawFuture),
                            Math.cos(absoluteFieldAngle - robotYawFuture));

            if (bestRPM != 0 && bestThetaHoodDegrees != 0) {
                canShoot(true);
                // We pass absoluteFieldAngle so the trajectory line points at the target
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        absoluteFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(bestRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        /**
         * MAKES THE PARABOLA FALL BACK TO THE MINIMUM HEIGHT
         *
         * @param shooterPose The pose of the shooter where the balls leave
         * @param targetPose The pose of the target
         * @param speeds The chassis speeds
         * @param currentRPM The current RPM of the shooter
         * @param maxHeightMeters The maximum height the ball can reach
         * @param minHeightMeters The minimum height the ball can reach
         * @param periodTime The loop time eg 20 ms
         * @return The calculated shooter state
         */
        public static ShooterState solve_max_and_min_iterative_with_vectors(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double minHeightMeters,
                double periodTime) {

            // 1. PROJECT ROBOT POSITION
            // Predict where the robot will be based on latency/processing time
            Pose2d predictedPose = Robot.pose_pred.getPredictedPose(shooterPose.toPose2d());
            FieldVisualizer.getInstance().updatePredictedPose(predictedPose);

            double sx = predictedPose.getX();
            double sy = predictedPose.getY();
            double sz = shooterPose.getZ();

            // 2. TARGET POSITION (Static Field Position + Air Resistance Offset)
            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx = targetPose.getX() + airResistanceAdder.getX();
            double ty = targetPose.getY() + airResistanceAdder.getY();
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            // Relative displacement to target
            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy);

            //     DogLog.log("ROBOT/Commands/Solve/Distance", floorDistance);

            // Iterative Search Variables
            double bestV_shooter = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestTurretFieldAngle = 0.0;
            double highestArc = -1.0;
            boolean foundValidShot = false;

            // 3. ITERATIVE ARC SEARCH
            // We test different apex heights to find a valid trajectory
            for (int i = 0; i < 5; i++) {
                double currentCeilingHeight =
                        minHeightMeters + (i * (maxHeightMeters - minHeightMeters) / 4.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                // Peak must be above both shooter and target
                if (hRise <= 0 || hFall <= 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                // VELOCITY THE BALL NEEDS (Field Frame)
                double vx_field = dx / totalTime;
                double vy_field = dy / totalTime;
                double vz_field = 9.81 * tRise;

                // 4. VECTOR SUBTRACTION (Shooting on the Move)
                // We subtract the robot's velocity so the launcher compensates for momentum
                double vx_shooter = vx_field - speeds.vxMetersPerSecond;
                double vy_shooter = vy_field - speeds.vyMetersPerSecond;
                double vz_shooter = vz_field; // Vertical velocity is independent of floor speeds

                double v_planar_shooter = Math.hypot(vx_shooter, vy_shooter);
                double total_v_shooter = Math.hypot(v_planar_shooter, vz_shooter);

                // Calculate Launcher Angles (Angle relative to the robot's hardware)
                double launchAngleDegrees =
                        Math.toDegrees(Math.atan2(vz_shooter, v_planar_shooter));
                double hoodAngleDegrees =
                        90.0 - launchAngleDegrees; // Assuming 0 is vertical, 90 is flat

                // 5. VALIDATE PHYSICAL CONSTRAINTS
                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) {
                    continue;
                }

                // Logic to pick the "Best" shot (here we prefer the highest arc for a 'lob')
                if (launchAngleDegrees > highestArc) {
                    highestArc = launchAngleDegrees;
                    bestV_shooter = total_v_shooter;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestTurretFieldAngle = Math.atan2(vy_shooter, vx_shooter);
                    foundValidShot = true;
                }
            }

            // 6. FINALIZE HARDWARE SETPOINTS
            double shooterRPM = 0.0;
            double turretAngle = 0.0;

            if (foundValidShot) {
                // Convert exit velocity to RPM (Using your 2.1 slip/recovery constant)
                shooterRPM =
                        (bestV_shooter / (2 * Math.PI * kFlywheels.WHEEL_RADIUS_METERS))
                                * 60
                                * EfficiencyConst;

                // Turret must point in the direction of the COMPENSATED vector
                double robotYaw = predictedPose.getRotation().getRadians();
                turretAngle =
                        Math.atan2(
                                Math.sin(bestTurretFieldAngle - robotYaw),
                                Math.cos(bestTurretFieldAngle - robotYaw));

                canShoot(true);
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV_shooter,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        bestTurretFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(shooterRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        /**
         * MAKES THE PARABOLA FALL BACK TO THE MINIMUM HEIGHT
         *
         * @param shooterPose The pose of the shooter where the balls leave
         * @param targetPose The pose of the target
         * @param speeds The chassis speeds
         * @param currentRPM The current RPM of the shooter
         * @param maxHeightMeters The maximum height the ball can reach
         * @param minHeightMeters The minimum height the ball can reach
         * @param periodTime The loop time eg 20 ms
         * @return The calculated shooter state
         */
        public static ShooterState solve_max_and_min_iterative_with_vectors_VARIED_MAX_HEIGHT(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double minHeightMeters,
                double periodTime) {

            // 1. PROJECT ROBOT POSITION
            // Predict where the robot will be based on latency/processing time
            Pose2d predictedPose = Robot.pose_pred.getPredictedPose(shooterPose.toPose2d());
            FieldVisualizer.getInstance().updatePredictedPose(predictedPose);

            double sx = predictedPose.getX();
            double sy = predictedPose.getY();
            double sz = shooterPose.getZ();

            // 2. TARGET POSITION (Static Field Position + Air Resistance Offset)
            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx = targetPose.getX() + airResistanceAdder.getX();
            double ty = targetPose.getY() + airResistanceAdder.getY();
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            // Relative displacement to target
            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy);

            // Iterative Search Variables
            double bestV_shooter = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestTurretFieldAngle = 0.0;
            double highestArc = -1.0;
            boolean foundValidShot = false;

            double maxHeight = maxHeightDistanceLerp.lerp(floorDistance);
            // 3. ITERATIVE ARC SEARCH
            // We test different apex heights to find a valid trajectory
            for (int i = 0; i < 5; i++) {
                double currentCeilingHeight = 1.0 + (i * (maxHeight - 1.0) / 4.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                // Peak must be above both shooter and target
                if (hRise <= 0 || hFall <= 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                // VELOCITY THE BALL NEEDS (Field Frame)
                double vx_field = dx / totalTime;
                double vy_field = dy / totalTime;
                double vz_field = 9.81 * tRise;

                // 4. VECTOR SUBTRACTION (Shooting on the Move)
                // We subtract the robot's velocity so the launcher compensates for momentum
                double vx_shooter = vx_field - speeds.vxMetersPerSecond;
                double vy_shooter = vy_field - speeds.vyMetersPerSecond;
                double vz_shooter = vz_field; // Vertical velocity is independent of floor speeds

                double v_planar_shooter = Math.hypot(vx_shooter, vy_shooter);
                double total_v_shooter = Math.hypot(v_planar_shooter, vz_shooter);

                // Calculate Launcher Angles (Angle relative to the robot's hardware)
                double launchAngleDegrees =
                        Math.toDegrees(Math.atan2(vz_shooter, v_planar_shooter));
                double hoodAngleDegrees =
                        90.0 - launchAngleDegrees; // Assuming 0 is vertical, 90 is flat

                // 5. VALIDATE PHYSICAL CONSTRAINTS
                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) {
                    continue;
                }

                // Logic to pick the "Best" shot (here we prefer the highest arc for a 'lob')
                if (launchAngleDegrees > highestArc) {
                    highestArc = launchAngleDegrees;
                    bestV_shooter = total_v_shooter;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestTurretFieldAngle = Math.atan2(vy_shooter, vx_shooter);
                    foundValidShot = true;
                }
            }

            // 6. FINALIZE HARDWARE SETPOINTS
            double shooterRPM = 0.0;
            double turretAngle = 0.0;

            if (foundValidShot) {
                // Convert exit velocity to RPM (Using your 2.1 slip/recovery constant)
                shooterRPM =
                        (bestV_shooter / (2 * Math.PI * kFlywheels.WHEEL_RADIUS_METERS))
                                * 60
                                * EfficiencyConst;

                // Turret must point in the direction of the COMPENSATED vector
                double robotYaw = predictedPose.getRotation().getRadians();
                turretAngle =
                        Math.atan2(
                                Math.sin(bestTurretFieldAngle - robotYaw),
                                Math.cos(bestTurretFieldAngle - robotYaw));

                canShoot(true);
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV_shooter,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        bestTurretFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(shooterRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        public static Translation2d addDToTargetWithAirResistance(
                Pose3d targetPose, Pose3d shooterPose, double launchAngleRads) {

            double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

            // Approximate Arc Length: If the shot is flat, length = d.
            // If it's a high arc, the path is significantly longer.
            // A simple trig approximation for path length: L = d / cos(launchAngle)
            double cosAngle = Math.cos(launchAngleRads);
            double arcLengthApproximation = d / Math.max(cosAngle, 0.5);

            double angleToTarget =
                    Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX());

            // Drag Constant: Adjust this '0.015' based on testing.
            // It adds extra distance to the 'target' to make the robot over-shoot
            // and compensate for drag.
            double dragComp = arcLengthApproximation * arcLengthApproximation * 0.015;

            double px = Math.cos(angleToTarget) * dragComp;
            double py = Math.sin(angleToTarget) * dragComp;

            return new Translation2d(px, py);
        }

        public static Translation2d addDToTargetWithAirResistance(
                Pose3d targetPose, Pose3d shooterPose) {
            double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

            Log.log("ROBOT/Commands/AimSolver/Distance", d);

            // this is complete bs
            double angleToTarget =
                    Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX());
            double p =
                    d
                            / airResistanceDTodistancedivider.lerp(
                                    d); // account for shots being close to missing when far. P
            // icrease as distance increases

            double px = Math.cos(angleToTarget) * p;
            double py = Math.sin(angleToTarget) * p;

            return new Translation2d(px, py);
        }

        public static double getShotTime(
                double ballLaunchVelocity,
                double hoodAngleRadians,
                double shooterHeight,
                double targetHeight) {
            // Vertical component of the velocity
            double vY = ballLaunchVelocity * Math.sin(hoodAngleRadians);

            // Time to reach the target height using the formula: h = vY * t - 0.5 * g * t^2
            // Rearranging gives: 0.5 * g * t^2 - vY * t + (targetHeight - shooterHeight) = 0
            double a = 0.5 * 9.81;
            double b = -vY;
            double c = targetHeight - shooterHeight;

            double discriminant = b * b - 4 * a * c;

            if (discriminant < 0) {
                // No real solution, the shot cannot reach the target height
                return Double.POSITIVE_INFINITY;
            }

            // We take the positive root since time cannot be negative
            double time = (-b + Math.sqrt(discriminant)) / (2 * a);
            return time;
        }

        public static void clearShotTrajectory() {
            Logger.recordOutput("Shooter/ShotTrajectory", new Translation2d[] {});
        }

        public static void publishShotTrajectory(
                double ballLaunchVelocity,
                double launchAngleRads, // Angle relative to the floor
                double fieldShotAngle, // Absolute angle toward the target
                Pose3d shooterPose3d,
                Pose3d targetPose3d) {

            double sx = shooterPose3d.getX();
            double sy = shooterPose3d.getY();
            double sz = shooterPose3d.getZ();

            // vZ is vertical, vH is horizontal across the floor
            double vZ = ballLaunchVelocity * Math.sin(launchAngleRads);
            double vH = ballLaunchVelocity * Math.cos(launchAngleRads);

            // Break horizontal velocity into field X and Y
            double vx = vH * Math.cos(fieldShotAngle);
            double vy = vH * Math.sin(fieldShotAngle);

            double time = getShotTime(ballLaunchVelocity, launchAngleRads, sz, targetPose3d.getZ());
            if (Double.isInfinite(time) || time <= 0) time = 1.5;

            int nPoints = 25;
            Pose3d[] trajectoryPoints = new Pose3d[nPoints];

            for (int i = 0; i < nPoints; i++) {
                double t = (time / (nPoints - 1)) * i;

                double x = sx + vx * t;
                double y = sy + vy * t;
                double z = sz + (vZ * t) - (0.5 * 9.81 * t * t);

                trajectoryPoints[i] = new Pose3d(x, y, Math.max(0, z), new Rotation3d());
            }

            Logger.recordOutput("Shooter/ShotTrajectory", trajectoryPoints);
        }
    }
}
