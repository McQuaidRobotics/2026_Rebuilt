package igknighters.constants;

import com.ctre.phoenix6.CANBus;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;

public class GeminiRobotConsts extends RobotConsts {

    public static final int firstMotorID = 15;
    public static final boolean disableAllLogs = false;
    public static final CANBus superStructure = new CANBus("SuperStructureBus");
    public static final CANBus drive = new CANBus("DriveBus");

    @Override
    public SWERVE_CONSTS swerve() {
        return new GeminiSwerveConsts();
    }

    public static class GeminiSwerveConsts implements SWERVE_CONSTS {
        static SwerveConsts swerveConsts = new SwerveConsts();

        static CommonSwerveConsts commonSwerveConsts = swerveConsts.getSwerveConsts();

        @Override
        public CommonSwerveConsts getCommonSwerveConsts() {
            return commonSwerveConsts;
        }
    }

    @Override
    public boolean disableAllLogs() {
        return disableAllLogs;
    }

    @Override
    public kLimelightVisionConsts limelightVision() {
        return new GeminiLimelightVisionConsts();
    }

    public static class GeminiLimelightVisionConsts implements kLimelightVisionConsts {
        @Override
        public String turretCam() {
            return "limelight";
        }

        @Override
        public String intakeCam() {
            return "limelight-intake";
        }

        @Override
        public String backCam() {
            return "limelight-back";
        }

        @Override
        public String rightCam() {
            return "limelight-left";
        }

        @Override
        public boolean disableVisionLogs() {
            return true;
        }
    }
}
