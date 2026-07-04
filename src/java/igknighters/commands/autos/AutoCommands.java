package igknighters.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;
import java.util.concurrent.atomic.AtomicBoolean;

public class AutoCommands {

    protected final Swerve swerve;
    protected final Subsystems subsystems;
    protected final AutoFactory autoFactory;

    public AutoCommands(Subsystems subsystems, AutoFactory factory) {
        this.swerve = subsystems.swerve;
        this.subsystems = subsystems;
        this.autoFactory = factory;
    }

    protected void logAutoEvent(String message, String event) {
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/Autos", message + " is " + event);
        }
    }

    protected Command loggedCmd(Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                logAutoEvent(this.getName(), "Started");
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                logAutoEvent(this.getName(), "Finished");
            }
        };
    }

    protected Boolean withinTolerance(Pose2d pose, Pose2d target, double tolerance) {
        return pose.getTranslation().getDistance(target.getTranslation()) < tolerance
                && Math.abs(pose.getRotation().getDegrees() - target.getRotation().getDegrees())
                        < tolerance;
    }

    protected double findSpeed(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    protected Trigger movingSlowerThan(Swerve swerve, double speed) {
        return new Trigger(() -> findSpeed(swerve.getState().Speeds) < speed);
    }

    protected class GenericAuto {
        private final AutoRoutine routine;
        private final ParallelCommandGroup headCommand = new ParallelCommandGroup();
        private final SequentialCommandGroup bodyCommand = new SequentialCommandGroup();
        private boolean trajectorybeenadded = false;

        private GenericAuto(AutoRoutine routine) {
            this.routine = routine;
        }

        public Command build() {
            System.out.println("Building auto: " + routine.toString());
            final AtomicBoolean flag = new AtomicBoolean(false);
            headCommand.addCommands(Commands.print(bodyCommand.getRequirements().toString()));
            bodyCommand.addCommands(
                    Commands.runOnce(() -> Log.log("ROBOT/Autos/ending the auto", true)),
                    new ScheduleCommand(Commands.runOnce(() -> flag.set(true))));
            routine.active()
                    .onTrue(
                            headCommand
                                    .andThen(
                                            Commands.print(
                                                    "HEAD COMMAND FINISHED - SHOULD HAVE RESET THE"
                                                            + " POSE"))
                                    .andThen(new ScheduleCommand(bodyCommand))
                                    .andThen(
                                            new ScheduleCommand(
                                                    Commands.print("BODY COMMAND IS SCHEDULED")))
                                    .withName(routine.toString() + "_AutoHead"));
            // routine.anyDone(null, null).onTrue(Commands.runOnce(() -> routine.reset()));
            return routine.cmd(flag::get);
        }

        private AutoTrajectory getTrajectory(Waypoints start, Waypoints end) {
            Trajectory<?> rawTraj = autoFactory.cache().loadTrajectory(start.to(end)).orElseThrow();
            return routine.trajectory(rawTraj);
        }

        // public Command intakeTrajectory()

        public GenericAuto addDrivingTrajectory(Waypoints start, Waypoints end, double timeout) {
            AutoTrajectory traj = getTrajectory(start, end);
            if (!trajectorybeenadded) {
                trajectorybeenadded = true;
                headCommand.addCommands(traj.resetOdometry().withTimeout(0.1));
            }
            bodyCommand.addCommands(
                    loggedCmd(traj.spawnCmd()).withName(traj.getRawTrajectory().name()));
            return this;
        }

        public GenericAuto addDrivingTrajectory(Waypoints... waypoints) {
            for (int i = 0; i < waypoints.length - 1; i += 1) {
                bodyCommand.addCommands(
                        getTrajectory(waypoints[i], waypoints[i + 1])
                                .spawnCmd()
                                .withName(
                                        "DRIVING FROM " + waypoints[i] + " TO " + waypoints[i + 1]),
                        SwerveCommands.stopDriving(swerve).withTimeout(.1));
            }
            return this;
        }
    }

    protected GenericAuto newRebuiltAuto(String name) {
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/Autos/Creation", "Creating new rebuilt auto: " + name);
        }
        return new GenericAuto(autoFactory.newRoutine(name));
    }
}
