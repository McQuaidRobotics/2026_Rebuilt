package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberState;

public class ClimberCommands {
    public static Command holdAtStateUntil(
            Climber climber, ClimberState state, double timeSeconds) {
        // ensures it gets to the state first, then holds it there for the specified time, then
        // stops the chainsaw
        return Commands.sequence(
                        goToState(climber, state),
                        holdAtState(climber, state).withTimeout(timeSeconds),
                        climber.runOnce(() -> climber.stopChainsaw()))
                .withName("HOLDING STATE: " + state.name() + " FOR " + timeSeconds + " SECONDS");
    }

    
    public static Command climbSequence(Climber climber) {
        return holdAtStateUntil(climber, ClimberState.CLIMB_PREP, 2.0)
                .andThen(holdAtStateUntil(climber, ClimberState.LATCH_ON, 1.0))
                .andThen(goToState(climber, ClimberState.PULL_UP))
                .withName("CLIMB SEQUENCE");
    }

    public static Command unClimb(Climber climber) {
        return goToState(climber, ClimberState.LATCH_ON)
                .andThen(goToState(climber, ClimberState.CLIMB_PREP));
    }

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
