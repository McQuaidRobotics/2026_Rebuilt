package igknighters.subsystems.shooter;

import igknighters.Robot;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.shooter.rollers.Rollers;
import igknighters.subsystems.shooter.rollers.RollersReal;
import igknighters.subsystems.shooter.rollers.RollersSim;

public class Shooter implements ExclusiveSubsystem {
    private final Rollers rollers;

    public Shooter() {
        if (Robot.isReal()) {
            rollers = new RollersReal();
        } else {
            rollers = new RollersSim();
        }
    }

    public void setRollerSpeed(double speedMetersPerSecond) {
        rollers.setSpeed(speedMetersPerSecond);
    }

    public void setRollerVoltage(double voltage) {
        rollers.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        rollers.periodic();
    }
}
