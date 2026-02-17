package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberState;

public class ClimberCommands {
    public static Command goUp(Climber climber) {
        return climber.run(climber::goUp)
                .until(climber::isUp)
                .finallyDo(() -> climber.stopChainsaw())
                .withName("GOING UP");
    }

    public static Command goDown(Climber climber) {
        return climber.run(climber::goDown)
                .until(climber::isDown)
                .finallyDo(() -> climber.stopChainsaw())
                .withName("GOING DOWN");
    }

    public static Command goToState(Climber climber, ClimberState state) {
        return climber.run(() -> climber.goToState(state))
                .until(
                        () -> {
                            if (state == ClimberState.CLIMB_PREP) return climber.isUp();
                            if (state == ClimberState.STOW) return climber.isDown();
                            if (state == ClimberState.PULL_UP) return climber.isDown();
                            return true;
                        })
                .finallyDo(() -> climber.stopChainsaw())
                .withName("GOING TO STATE: " + state.name());
    }

    public static Command holdAtState(Climber climber, ClimberState state) {
        return climber.run(() -> climber.goToState(state))
                .withName("HOLDING STATE: " + state.name());
    }

    public static Command holdDown(Climber climber) {
        return climber.run(climber::goDown).withName("HOLDING DOWN");
    }

    public static Command holdUp(Climber climber) {
        return climber.run(climber::goUp).withName("HOLDING UP");
    }

    public static Command stop(Climber climber) {
        return climber.runOnce(climber::stopChainsaw).withName("STOP CLIMBER");
    }
}
