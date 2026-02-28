package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberState;

public class ClimberCommands {

    /**
     * Goes to a state and holds it for a certain amount of time, then stops the climber
     *
     * @param climber
     * @param state
     * @param timeSeconds
     * @return A command that goes to the specified state, holds it for the specified time, and then
     *     stops the climber
     */
    public static Command holdAtStateUntil(
            Climber climber, ClimberState state, double timeSeconds) {

        return Commands.sequence(
                        goToState(climber, state),
                        holdAtState(climber, state).withTimeout(timeSeconds),
                        climber.runOnce(() -> climber.stopChainsaw()))
                .withName("HOLDING STATE: " + state.name() + " FOR " + timeSeconds + " SECONDS");
    }

    /**
     * Goes through the climb sequence: Climb Prep -> Latch On -> Pull Up, holding Pull up until the
     * command is interrupted. Climb Prep is held for 2 seconds, and Latch On is held for 1 second.
     *
     * @param climber
     * @return A command that goes through the climb sequence: Climb Prep -> Latch On -> Pull Up,
     *     holding Pull up until the command is interrupted. Climb Prep is held for 2 seconds, and
     *     Latch On is held for 1 second.
     */
    public static Command climbSequence(Climber climber) {
        return holdAtStateUntil(climber, ClimberState.LATCH_ON, 2.0)
                .andThen(holdAtState(climber, ClimberState.PULL_UP))
                .withName("CLIMB SEQUENCE");
    }

    /**
     * Goes through the unclimb sequence: Pull Up -> Latch On -> Climb Prep -> Stow. Ends when
     * climber reaches state of climb prep
     *
     * @param climber
     * @return A command that goes through the unclimb sequence: Pull Up -> Latch On -> Climb Prep
     *     -> Stow. Ends when climber reaches state of climb prep
     */
    public static Command unClimb(Climber climber) {
        return goToState(climber, ClimberState.LATCH_ON)
                .andThen(goToState(climber, ClimberState.CLIMB_PREP))
                .andThen(goToState(climber, ClimberState.STOW))
                .withName("UNCLIMB SEQUENCE");
    }

    /**
     * Chainsaw up until the climber is up, then stops the climber
     *
     * @param climber
     * @return A command that goes up until the Chainsaw is up, then stops the climber
     */
    public static Command goUp(Climber climber) {
        return climber.run(climber::goUp)
                .until(climber::isUp)
                .finallyDo(() -> climber.stopChainsaw())
                .withName("GOING UP");
    }

    /**
     * Chainsaw down until the climber is down, then stops the climber
     *
     * @param climber
     * @return A command that goes down until the Chainsaw is down, then stops the climber
     */
    public static Command goDown(Climber climber) {
        return climber.run(climber::goDown)
                .until(climber::isDown)
                .finallyDo(() -> climber.stopChainsaw())
                .withName("GOING DOWN");
    }

    /**
     * Goes to a state and holds it until the command is interrupted, then stops the climber
     *
     * @param climber
     * @param state
     * @return A command that goes to the specified state, holds it until the command is
     *     interrupted, and then stops the climber
     */
    public static Command goToState(Climber climber, ClimberState state) {
        return climber.run(() -> climber.goToState(state))
                .until(
                        () -> {
                            switch (state) {
                                case CLIMB_PREP:
                                case LATCH_ON:
                                    return climber.isUp();
                                case STOW:
                                    return climber.isDown();
                                case PULL_UP:
                                    return climber.isMiddle();
                                default:
                                    return true;
                            }
                        })
                .finallyDo(() -> climber.stopChainsaw())
                .withName("GOING TO STATE: " + state.name());
    }

    /**
     * Holds a state until the command is interrupted, then stops the climber
     *
     * @param climber
     * @param state
     * @return A command that holds the specified state until the command is interrupted, and then
     *     stops the climber
     */
    public static Command holdAtState(Climber climber, ClimberState state) {
        return climber.run(() -> climber.goToState(state))
                .withName("HOLDING STATE: " + state.name());
    }

    /**
     * Holds the chainsaw down until the command is interrupted, then stops the climber
     *
     * @param climber
     * @return A command that holds the chainsaw down until the command is interrupted, and then
     *     stops the climber
     */
    public static Command holdDown(Climber climber) {
        return climber.run(climber::goDown).withName("HOLDING DOWN");
    }

    /**
     * Holds the chainsaw up until the command is interrupted, then stops the climber
     *
     * @param climber
     * @return A command that holds the chainsaw up until the command is interrupted, and then stops
     *     the climber
     */
    public static Command holdUp(Climber climber) {
        return climber.run(climber::goUp).withName("HOLDING UP");
    }

    /**
     * Stops the climber
     *
     * @param climber
     * @return A command that stops the climber
     */
    public static Command stop(Climber climber) {
        return climber.runOnce(climber::stopChainsaw).withName("STOP CLIMBER");
    }
}
