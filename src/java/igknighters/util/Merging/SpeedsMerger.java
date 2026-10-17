package igknighters.util.Merging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SpeedsMerger {
    public static ChassisSpeeds simpleMerge(
            ChassisSpeeds speeds1, ChassisSpeeds speeds2, double weight1, double weight2) {
        return new ChassisSpeeds(
                (speeds1.vxMetersPerSecond * weight1 + speeds2.vxMetersPerSecond * weight2)
                        / (weight1 + weight2),
                (speeds1.vyMetersPerSecond * weight1 + speeds2.vyMetersPerSecond * weight2)
                        / (weight1 + weight2),
                (speeds1.omegaRadiansPerSecond * weight1 + speeds2.omegaRadiansPerSecond * weight2)
                        / (weight1 + weight2));
    }

    public static ChassisSpeeds trustedMerge(ChassisSpeeds speeds1, ChassisSpeeds speeds2) {
        double vx1 = speeds1.vxMetersPerSecond;
        double vy1 = speeds1.vyMetersPerSecond;
        double omega1 = speeds1.omegaRadiansPerSecond;

        double vx2 = speeds2.vxMetersPerSecond;
        double vy2 = speeds2.vyMetersPerSecond;
        double omega2 = speeds2.omegaRadiansPerSecond;

        double dvx = vx2 - vx1;
        double dvy = vy2 - vy1;
        double domega = omega2 - omega1;

        double modifierVx = dvx * Math.exp(-Math.abs(dvx));
        double modifierVy = dvy * Math.exp(-Math.abs(dvy));
        double modifierOmega = domega * Math.exp(-Math.abs(domega));

        // Simple trusted merge: give more weight to the first set of speeds
        return new ChassisSpeeds(vx1 + modifierVx, vy1 + modifierVy, omega1 + modifierOmega);
    }
}
