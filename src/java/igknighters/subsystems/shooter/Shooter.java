package igknighters.subsystems.shooter;

import dev.doglog.DogLog;
import igknighters.Robot;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.shooter.rollers.Rollers;
import igknighters.subsystems.shooter.rollers.RollersReal;
import igknighters.subsystems.shooter.rollers.RollersSim;
import igknighters.subsystems.shooter.turret.Turret;
import igknighters.subsystems.shooter.turret.TurretReal;
import igknighters.subsystems.shooter.turret.TurretSim;

public class Shooter implements ExclusiveSubsystem {
    private final Rollers rollers;
    private final Turret turret;

    public Shooter() {
        if (Robot.isReal()) {
            rollers = new RollersReal();
            turret = new TurretReal();
        } else {
            rollers = new RollersSim();
            turret = new TurretSim();
        }
    }

    private void targetSpeed(double speedRPM) {
        rollers.setSpeed(speedRPM);
    }

    private void setTurretAngleDegrees(double angleDegrees) {
        turret.setAngleDegrees(angleDegrees);
    }

    private double getTurretAngleDegrees() {
        return turret.getAngleDegrees();
    }

    private void goToTurretAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/TARGETING", angleDegrees);
        turret.goToAngleDegrees(angleDegrees);
    }

    public void targetState(double rpm, double angleDegrees) {
        DogLog.log("Subsystems/Shooter/TARGETING/RPM", rpm);
        DogLog.log("Subsystems/Shooter/TARGETING/ANGLE", angleDegrees);
        targetSpeed(rpm);
        goToTurretAngleDegrees(angleDegrees);
    }

    public void setTurretPosition(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/SETSTATE/ANGLE", angleDegrees);
        turret.setAngleDegrees(angleDegrees);
    }

    public void setRollerVoltage(double voltage) {
        rollers.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        rollers.periodic();
        turret.periodic();
    }
}
