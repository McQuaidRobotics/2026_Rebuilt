package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.AbleToShootSharedState;
import igknighters.subsystems.shooter.flywheel.Flywheel;
import igknighters.subsystems.shooter.flywheel.FlywheelDisabled;
import igknighters.subsystems.shooter.flywheel.FlywheelSimulator;
import igknighters.subsystems.shooter.hood.Hood;
import igknighters.subsystems.shooter.hood.HoodDisabled;
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
            hood = new HoodDisabled();
        } else {
            rollers = new FlywheelSimulator();
            turret = new TurretSim();
            hood = new HoodSim();
        }
        visualizer = new ShooterVisualizer();
    }

    private void targetSpeed(AngularVelocity velo) {
        rollers.setSpeed(velo);
    }

    public double getHoodAngleDegrees() {
        return hood.getAngleDegrees();
    }

    public void setTurretAngleDegrees(Angle angle) {
        turret.setAngle(angle);
    }

    public double getTurretAngleDegrees() {
        return turret.getAngleDegrees();
    }

    private void goToTurretAngle(Angle angle) {
        DogLog.log("Subsystems/Shooter/TARGETING", angle.in(Degrees));
        turret.goToAngleDegrees(angle);
    }

    public void targetState(AngularVelocity velo, Angle turretAngle, Angle hoodAngle) {
        DogLog.log("Subsystems/Shooter/TARGETING/RPM", velo.in(RPM));
        DogLog.log("Subsystems/Shooter/TARGETING/ANGLE", turretAngle.in(Degrees));
        DogLog.log("Subsystems/Shooter/TARGETING/HoodAngle", hoodAngle.in(Degrees));
        targetSpeed(velo);
        goToTurretAngle(turretAngle);
        hood.goToAngle(hoodAngle);
        goalRPM = velo.in(RPM);
        goalTurretAngleDegrees = turretAngle.in(Degrees);
        goalHoodAngleDegrees = hoodAngle.in(Degrees);
    }

    public boolean atTarget(
            double toleranceRPM, double toleranceDegrees, double toleranceHoodDegrees) {
        boolean atSpeed = Math.abs(rollers.getSpeed().in(RPM) - goalRPM) < toleranceRPM;
        boolean atTurretAngle =
                Math.abs(getTurretAngleDegrees() - goalTurretAngleDegrees) < toleranceDegrees;
        boolean atHoodAngle =
                Math.abs(hood.getAngleDegrees() - goalHoodAngleDegrees) < toleranceHoodDegrees;
        DogLog.log("Subsystems/Shooter/AT TARGET/AT SPEED", atSpeed);
        DogLog.log("Subsystems/Shooter/AT TARGET/AT TURRET ANGLE", atTurretAngle);
        DogLog.log("Subsystems/Shooter/AT TARGET/AT HOOD ANGLE", atHoodAngle);
        return atSpeed && atTurretAngle && atHoodAngle;
    }

    public void setTurretPosition(Angle angle) {
        DogLog.log("Subsystems/Shooter/SETSTATE/ANGLE", angle.in(Degrees));
        turret.setAngle(angle);
    }

    public ShooterState getCurrentState() {
        return new ShooterState(
                rollers.getSpeed(),
                Degrees.of(turret.getAngleDegrees()),
                Degrees.of(hood.getAngleDegrees()));
    }

    public void setRollerVoltage(double voltage) {
        rollers.setVoltage(voltage);
    }

    public double getEstimatedRPM(double distanceMeters) {
        return rpmTable.lerp(distanceMeters);
    }

    public void setHoodAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/SETSTATE/HoodAngle", angleDegrees);
        hood.setAngle(angleDegrees);
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
