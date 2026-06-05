package igknighters.subsystems.YamShooter;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ShooterStatusIndicators {
    private final Mechanism2d indicator = new Mechanism2d(30, 10);
    private final MechanismRoot2d RPMROOT = indicator.getRoot("RPM INDICATOR", 5, 0);
    private final MechanismRoot2d TURRETROOT = indicator.getRoot("TURRET INDICATOR", 15, 0);
    private final MechanismRoot2d HOODROOT = indicator.getRoot("HOOD INDICATOR", 25, 0);
    private final MechanismLigament2d rpm =
            RPMROOT.append(new MechanismLigament2d("IDK BROCHACO", 10, 90));
    private final MechanismLigament2d turret =
            TURRETROOT.append(new MechanismLigament2d("IDK BROCHACO1", 10, 90));
    private final MechanismLigament2d hood =
            HOODROOT.append(new MechanismLigament2d("IDK BROCHACO2", 10, 90));

    public ShooterStatusIndicators() {
        indicator.setBackgroundColor(new Color8Bit(Color.kBlack));
        rpm.setLineWeight(5);
        turret.setLineWeight(5);
        hood.setLineWeight(5);

        SmartDashboard.putData("VISUALIZERS/SHOOTER/STATUS", indicator);
    }

    public void setThingyBasedOnBoolean(MechanismLigament2d thingy, boolean bool) {
        if (bool) {
            thingy.setColor(new Color8Bit(Color.kGreen));
        } else {
            thingy.setColor(new Color8Bit(Color.kRed));
        }
    }

    public void update(boolean atRPM, boolean atTurretAngle, boolean atHoodAngle) {
        setThingyBasedOnBoolean(rpm, atRPM);
        setThingyBasedOnBoolean(turret, atTurretAngle);
        setThingyBasedOnBoolean(hood, atHoodAngle);
    }
}
