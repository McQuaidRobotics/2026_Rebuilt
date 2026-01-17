package igknighters.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.HigherOrderCommands;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public class AutoCommands {

    protected final CommandSwerveDrivetrain swerve;
    protected final Subsystems subsystems;
    protected final AutoFactory autoFactory;

    public AutoCommands(Subsystems subsystems, AutoFactory factory) {
        this.swerve = subsystems.swerve;
        this.subsystems = subsystems;
        this.autoFactory = factory;
    }

    protected void logAutoEvent(String message, String event) {
        DogLog.log("Robot/Commands/Autos", message + " is " + event);
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

    protected Trigger movingSlowerThan(CommandSwerveDrivetrain swerve, double speed) {
        return new Trigger(() -> findSpeed(swerve.getState().Speeds) < speed);
    }

    protected class RebuiltAuto {
        private final AutoRoutine routine;
        private final ParallelCommandGroup headCommand = new ParallelCommandGroup();
        private final SequentialCommandGroup bodyCommand = new SequentialCommandGroup();
        private boolean trajectorybeenadded = false;

        private RebuiltAuto(AutoRoutine routine) {
            this.routine = routine;
        }

        public Command build() {
            final AtomicBoolean flag = new AtomicBoolean(false);
            headCommand.addCommands(Commands.print(bodyCommand.getRequirements().toString()));
            bodyCommand.addCommands(
                    Commands.runOnce(() -> DogLog.log("Robot/Autos/ending the auto", true)),
                    new ScheduleCommand(Commands.runOnce(() -> flag.set(true))));
            routine.active()
                    .onTrue(
                            headCommand
                                    .andThen(new ScheduleCommand(bodyCommand))
                                    .andThen(
                                            new ScheduleCommand(
                                                    Commands.print("BODY COMMAND IS SCHEDULED")))
                                    .withName(routine.toString() + "_AutoHead"));
            return routine.cmd(flag::get);
        }

        private AutoTrajectory getTrajectory(Waypoints start, Waypoints end) {

            Trajectory<?> rawTraj = autoFactory.cache().loadTrajectory(start.to(end)).orElseThrow();
            return routine.trajectory(rawTraj);
            // HOW TO HANDLE MIRRORING ACROSS X such that (x, y, theta) becomes (x, FIELD_WIDTH - y,
            // -theta)
        }

        // public Command intakeTrajectory()

        public RebuiltAuto shootThenMove(Waypoints start, Waypoints end, double timeout) {
            AutoTrajectory traj = getTrajectory(start, end);
            if (!trajectorybeenadded) {
                trajectorybeenadded = true;
                headCommand.addCommands(traj.resetOdometry().withTimeout(0.1));
            }
            bodyCommand.addCommands(
                    loggedCmd(
                            Commands.sequence(
                                            HigherOrderCommands.shootTillEmpty(subsystems, timeout),
                                            traj.cmd(),
                                            SwerveCommands.stopDriving(swerve))
                                    .withName(traj.getRawTrajectory().name())));
            return this;
        }

        public RebuiltAuto shootAndMove(Waypoints start, Waypoints end) {
            AutoTrajectory trajectory = getTrajectory(start, end);
            if (!trajectorybeenadded) {
                trajectorybeenadded = true;
                headCommand.addCommands(trajectory.resetOdometry().withTimeout(0.1));
            }

            bodyCommand.addCommands(
                    loggedCmd(
                            Commands.sequence(
                                            Commands.parallel(
                                                    HigherOrderCommands.shootTillEmpty(
                                                            subsystems, 3.0),
                                                    trajectory.cmd()),
                                            SwerveCommands.stopDriving(swerve))
                                    .withName(trajectory.getRawTrajectory().name())));
            return this;
        }

        public RebuiltAuto addDrivingTrajectory(Waypoints... waypoints) {
            for (int i = 0; i < waypoints.length - 1; i += 1) {
                bodyCommand.addCommands(
                        getTrajectory(waypoints[i], waypoints[i + 1])
                                .cmd()
                                .withName(
                                        "DRIVING FROM " + waypoints[i] + " TO " + waypoints[i + 1]),
                        SwerveCommands.stopDriving(swerve).withTimeout(3.0));
            }
            return this;
        }
    }

    protected class GenericAuto {
        private final AutoRoutine routine;
        private final ParallelCommandGroup headCommand = new ParallelCommandGroup();
        private final SequentialCommandGroup bodyCommand = new SequentialCommandGroup();
        private boolean trajectorybeenadded = false;

        private GenericAuto(AutoRoutine routine) {
            this.routine = routine;
        }

        private AutoTrajectory getTrajectory(Waypoints start, Waypoints end) {

            Trajectory<?> rawTraj = autoFactory.cache().loadTrajectory(start.to(end)).orElseThrow();
            return routine.trajectory(rawTraj);
        }

        private Command finishAlignment(AutoTrajectory trajectory, double distOffset) {
            if (trajectory.getFinalPose().isPresent()) {
                Supplier<Command> cmdSup =
                        () -> {
                            final Pose2d finalPose =
                                    trajectory
                                            .getFinalPose()
                                            .get()
                                            .plus(new Transform2d(distOffset, 0, Rotation2d.kZero));
                            return loggedCmd(
                                    SwerveCommands.moveToSimple(swerve, finalPose)
                                            .until(
                                                    () ->
                                                            withinTolerance(
                                                                            SwerveCommands.getPose(
                                                                                    swerve),
                                                                            finalPose,
                                                                            0.1)
                                                                    && movingSlowerThan(swerve, .08)
                                                                            .getAsBoolean()));
                        };
                return Commands.defer(cmdSup, Set.of(swerve));
            } else {
                DriverStation.reportError("NO FINAL POSE IN THE AUTO ROUTINE", false);
                return Commands.none();
            }
        }

        public GenericAuto addDrivingTrajectory(Waypoints... waypoints) {
            headCommand.addCommands(
                    getTrajectory(waypoints[0], waypoints[1]).resetOdometry().withTimeout(0.1));
            for (int i = 0; i < waypoints.length - 1; i += 1) {
                bodyCommand.addCommands(
                        getTrajectory(waypoints[i], waypoints[i + 1]).cmd(),
                        finishAlignment(getTrajectory(waypoints[i], waypoints[i + 1]), 0.0)
                                .withName("FINISHING - ALIGNMENT"),
                        SwerveCommands.stopDriving(swerve).withTimeout(3.0));
            }
            return this;
        }

        public Command build() {
            final AtomicBoolean flag = new AtomicBoolean(false);
            headCommand.addCommands(Commands.print(bodyCommand.getRequirements().toString()));
            bodyCommand.addCommands(
                    Commands.runOnce(() -> DogLog.log("Robot/Autos/ending the auto", true)),
                    new ScheduleCommand(Commands.runOnce(() -> flag.set(true))));
            routine.active()
                    .onTrue(
                            headCommand
                                    .andThen(new ScheduleCommand(bodyCommand))
                                    .andThen(
                                            new ScheduleCommand(
                                                    Commands.print("BODY COMMAND IS SCHEDULED")))
                                    .withName(routine.toString() + "_AutoHead"));
            return routine.cmd(flag::get);
        }
    }

    protected GenericAuto newAuto(String name) {
        DogLog.log("Robot/Commands/Autos/Creation", "Creating new auto: " + name);
        return new GenericAuto(autoFactory.newRoutine(name));
    }

    protected RebuiltAuto newRebuiltAuto(String name) {
        DogLog.log("Robot/Commands/Autos/Creation", "Creating new rebuilt auto: " + name);
        return new RebuiltAuto(autoFactory.newRoutine(name));
    }
}
