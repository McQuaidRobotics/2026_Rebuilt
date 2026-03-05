package igknighters.subsystems.swerve.swerveconstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public abstract class CommonSwerveConsts {
    public abstract CommandSwerveDrivetrain createDrivetrain(Subsystem requirement);

    public abstract double getMaxSpeedMetersPerSecond();

    public abstract SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            getFrontLeftModule();

    public abstract SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            getFrontRightModule();

    public abstract SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            getBackLeftModule();

    public abstract SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            getBackRightModule();
}
