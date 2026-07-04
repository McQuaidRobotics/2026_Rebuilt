package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.Swerve;

public class Subsystems {
    public final Swerve swerve;
    public final LimeLightVision vision;
    public final Led led;

    public final SubsystemBase[] lockedResources;

    public Subsystems(Swerve swerve, LimeLightVision vision, Led led) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.lockedResources = new SubsystemBase[] {swerve, vision, led};
    }
}
