package igknighters.commands.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.turret.Turret;

public class shooterComands {
    public static Command setAngle(Turret turret,Angle angle){
        return turret.run(()-> turret.targetAngle(angle));
    }
    
}
