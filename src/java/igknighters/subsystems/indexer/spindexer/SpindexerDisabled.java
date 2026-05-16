package igknighters.subsystems.indexer.spindexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SpindexerDisabled extends SpindexerBase {

    @Override
    AngularVelocity getVelocity() {
        return RPM.of(0);
    }
    @Override
    Command setVelocity(AngularVelocity speed) {
        return Commands.none();
    }
    @Override
    void setVelocitySetpoint(AngularVelocity speed) {
        
    }
}
