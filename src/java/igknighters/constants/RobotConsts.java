package igknighters.constants;

import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;

public abstract class RobotConsts {

    public abstract SWERVE_CONSTS swerve();

    public abstract boolean disableAllLogs();

    public abstract kLimelightVisionConsts limelightVision();

    public interface SWERVE_CONSTS {
        CommonSwerveConsts getCommonSwerveConsts();
    }

    public interface kLimelightVisionConsts {
        String turretCam();

        String intakeCam();

        String backCam();

        String rightCam();

        boolean disableVisionLogs();
    }
}
