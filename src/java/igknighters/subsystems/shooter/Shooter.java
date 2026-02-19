package igknighters.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.AbleToShootSharedState;
import igknighters.constants.Conv;
import igknighters.subsystems.shooter.flywheel.Flywheel;
import igknighters.subsystems.shooter.flywheel.FlywheelDisabled;
import igknighters.subsystems.shooter.flywheel.FlywheelSimulator;
import igknighters.subsystems.shooter.hood.Hood;
import igknighters.subsystems.shooter.hood.HoodReal;
import igknighters.subsystems.shooter.hood.HoodSim;
import igknighters.subsystems.shooter.turret.Turret;
import igknighters.subsystems.shooter.turret.TurretDisabled;
import igknighters.subsystems.shooter.turret.TurretSim;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;

public class Shooter extends SubsystemBase {
    private final Flywheel rollers;
    private final Turret turret;
    private final Hood hood;
    private final ShooterVisualizer visualizer;
    private AbleToShootSharedState ableToShootState = AbleToShootSharedState.getInstance();
    private double goalRPM = 100.0;
    private double goalTurretAngleDegrees = 10.0;
    private double goalHoodAngleDegrees = 10.0;
    private LerpTable rpmTable =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.0, 2800.0),
                        new LerpTableEntry(3.0, 3000.0),
                        new LerpTableEntry(5.0, 4000.0),
                        new LerpTableEntry(10.0, 4500.0),
                        new LerpTableEntry(15.0, 5500.0),
                        new LerpTableEntry(20.0, 6000.0),
                    });

    public Shooter() {
        if (Robot.isReal()) {
            rollers = new FlywheelDisabled();
            turret = new TurretDisabled();
            hood = new HoodReal();
        } else {
            rollers = new FlywheelSimulator();
            turret = new TurretSim();
            hood = new HoodSim();
        }
        visualizer = new ShooterVisualizer();
    }

    private void targetSpeed(double speedRPM) {
        rollers.setSpeed(speedRPM);
    }

    public double getHoodAngleDegrees() {
        return hood.getAngleDegrees();
    }

    private void setTurretAngleDegrees(double angleDegrees) {
        turret.setAngleDegrees(angleDegrees);
    }

    public double getTurretAngleDegrees() {
        return turret.getAngleDegrees();
    }

    private void goToTurretAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/TARGETING", angleDegrees);
        turret.goToAngleDegrees(angleDegrees);
    }

    public void targetState(double rpm, double turretAngleDegrees, double hoodAngleRads) {
        DogLog.log("Subsystems/Shooter/TARGETING/RPM", rpm);
        DogLog.log("Subsystems/Shooter/TARGETING/ANGLE", turretAngleDegrees);
        DogLog.log("Subsystems/Shooter/TARGETING/HoodAngle", hoodAngleRads);
        targetSpeed(rpm);
        goToTurretAngleDegrees(turretAngleDegrees);
        hood.goToAngleDegrees(hoodAngleRads * Conv.RADIANS_TO_DEGREES);
        goalRPM = rpm;
        goalTurretAngleDegrees = turretAngleDegrees;
        goalHoodAngleDegrees = hoodAngleRads;
    }

    public boolean atTarget(
            double toleranceRPM, double toleranceDegrees, double toleranceHoodDegrees) {
        boolean atSpeed = Math.abs(rollers.getSpeedRPM() - goalRPM) < toleranceRPM;
        boolean atTurretAngle =
                Math.abs(getTurretAngleDegrees() - goalTurretAngleDegrees) < toleranceDegrees;
        boolean atHoodAngle =
                Math.abs(hood.getAngleDegrees() - goalHoodAngleDegrees) < toleranceHoodDegrees;
        DogLog.log("Subsystems/Shooter/AT TARGET/AT SPEED", atSpeed);
        DogLog.log("Subsystems/Shooter/AT TARGET/AT TURRET ANGLE", atTurretAngle);
        DogLog.log("Subsystems/Shooter/AT TARGET/AT HOOD ANGLE", atHoodAngle);
        return atSpeed && atTurretAngle && atHoodAngle;
    }

    public void setTurretPosition(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/SETSTATE/ANGLE", angleDegrees);
        turret.setAngleDegrees(angleDegrees);
    }

    public ShooterState getCurrentState() {
        return new ShooterState(
                rollers.getSpeedRPM(),
                turret.getAngleDegrees() * Conv.DEGREES_TO_RADIANS,
                hood.getAngleDegrees() * Conv.DEGREES_TO_RADIANS);
    }

    public void setRollerVoltage(double voltage) {
        rollers.setVoltage(voltage);
    }

    public double getEstimatedRPM(double distanceMeters) {
        return rpmTable.lerp(distanceMeters);
    }

    public void setHoodAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/SETSTATE/HoodAngle", angleDegrees);
        hood.setAngleDegrees(angleDegrees);
    }

    @Override
    public void periodic() {
        rollers.periodic();
        turret.periodic();
        hood.periodic();

        visualizer.update(getCurrentState(), goalRPM, goalHoodAngleDegrees);
        ableToShootState.setCanShoot(atTarget(600, 1, 5));
    }
}
