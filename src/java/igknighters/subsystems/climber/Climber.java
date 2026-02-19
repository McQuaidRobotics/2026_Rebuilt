package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.climber.chainsaw.Chainsaw;
import igknighters.subsystems.climber.chainsaw.ChainsawDisabled;
import igknighters.subsystems.climber.chainsaw.ChainsawSim;
import igknighters.subsystems.climber.servos.Servos;
import igknighters.subsystems.climber.servos.ServosDisabled;
import igknighters.subsystems.climber.servos.ServosSim;

public class Climber extends SubsystemBase {
    private Chainsaw chainsaw;
    private Servos servos;
    private boolean usingRealSensor;

    public Climber() {
        if (Robot.isReal()) {
            chainsaw = new ChainsawDisabled();
            servos = new ServosDisabled();
            usingRealSensor = true;
        } else {
            chainsaw = new ChainsawSim();
            servos = new ServosSim();
            usingRealSensor = false;
        }
    }

    public double getPositionInches() {
        return chainsaw.getPositionInches();
    }

    public void setPositionInches(double position) {
        chainsaw.setPositionInches(position);
    }

    public void goToInches(double inches) {
        chainsaw.goToInches(inches);
    }

    public void goToState(ClimberState state) {
        chainsaw.goToInches(state.targetHeightInches);
        servos.goToAngleDegrees(state.movingClimberServosDeployed, Servos.ServoID.MOVING_SERVOS);
        servos.goToAngleDegrees(state.stationaryClimberServosDeployed, Servos.ServoID.FIXED_SERVOS);
    }

    public boolean isAt(double targetInches, double toleranceInches) {
        return Math.abs(getPositionInches() - targetInches) <= toleranceInches;
    }

    public boolean isSensorHit() {
        return chainsaw.isSensorHit();
    }

    @Override
    public void periodic() {
        chainsaw.periodic();
    }
}
