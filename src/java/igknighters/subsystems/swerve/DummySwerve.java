package igknighters.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySwerve extends SubsystemBase {
    public Command doNothing(){
        return Commands.none();
    }
}

