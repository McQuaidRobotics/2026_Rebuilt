package igknighters.subsystems.climber.servos;

import dev.doglog.DogLog;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import edu.wpi.first.wpilibj.Servo;

public class ServosReal extends Servos {
    
    private final Servo UpperServos;

    public ServosReal(){
        UpperServos = new Servo(SubsystemConstants.kClimber.kServos.SERVO_PORT);
        //we still need to define the constants

    }

    double desiredAngleUpperServos = 0.0;

    @Override
    public void setAngleDegrees(double angleDegrees) {
        desiredAngleUpperServos = angleDegrees;
    }

    @Override
    public double getAngleDegrees() {
        return UpperServos.getAngle();
        //if there are other servos, those need to be added
    }

    @Override
    public void periodic() {
        DogLog.log("Subsystems/Climber/Servos/Position", getAngleDegrees());
        UpperServos.setAngle(desiredAngleUpperServos);
        //the angle needs to be set periodically, or the servo will get angry and turn off after 100ms
    }

    @Override
    public boolean isAt(double targetAngleDegrees, double toleranceDegrees) {
        // NOT done yet
        return Math.abs(getAngleDegrees()-targetAngleDegrees) < toleranceDegrees;
    }
    
}
