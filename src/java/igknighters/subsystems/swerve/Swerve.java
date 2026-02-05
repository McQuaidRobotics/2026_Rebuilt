package igknighters.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import igknighters.Robot;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;

public class Swerve {
    CommandSwerveDrivetrain drivetrain;
    SwerveConsts swerveConsts = new SwerveConsts();
    boolean isSwerveDisabled = false;
    DummySwerve dummySwerve = new DummySwerve();

    public Swerve(){
        if(!isSwerveDisabled){
            drivetrain = swerveConsts.getSwerveConsts().createDrivetrain();
        }
    }

    public void periodic(){
        if(!isSwerveDisabled){
            drivetrain.periodic();
        }
    }

    public void followPath(SwerveSample sample){
        if (!isSwerveDisabled) {
            drivetrain.followPath(sample);
        }
    }

    public void resetPose(Pose2d pose){
        if (!isSwerveDisabled) {
            drivetrain.resetPose(pose);
        }
    }

    public AutoFactory createAutoFactory() {
        if (!isSwerveDisabled) {
            return drivetrain.createAutoFactory();
        } else {
            return new AutoFactory(() -> new Pose2d(), this::resetPose, this::followPath, true, dummySwerve);
        }
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> logger) {
        if (!isSwerveDisabled) {
            return drivetrain.createAutoFactory(logger);
        } else {
            return new AutoFactory(() -> new Pose2d(), this::resetPose, this::followPath, true, new DummySwerve());
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        if (!isSwerveDisabled) {
            return drivetrain.applyRequest(requestSupplier);
        } else {
            return dummySwerve.doNothing();
        }
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        if (!isSwerveDisabled) {
            return drivetrain.sysIdQuasistatic(direction);
        } else {
            return dummySwerve.doNothing();
        }
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if (!isSwerveDisabled) {
            return drivetrain.sysIdDynamic(direction);
        } else {
            return dummySwerve.doNothing();
        }
    }
    public void addVisionMeasurement(Pose2d visionPose, double timestamp){
        if (!isSwerveDisabled) {
            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }
    }
    
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if (!isSwerveDisabled) {
        drivetrain.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
        }
    }
    

    
}
