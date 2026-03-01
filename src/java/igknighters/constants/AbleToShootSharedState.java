package igknighters.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.util.log.Log;

/**
 * Shared state to determine if the shooter is able to shoot. This class acts as a "lockless"
 * resource, accessible by multiple commands/loops simultaneously. Implemented as a singleton, its
 * 'canShoot' state is updated externally.
 */
public class AbleToShootSharedState {
    private static AbleToShootSharedState instance;

    private boolean atTarget = false;
    private boolean beingControlled = false;
    private boolean possibleShot = false;
    private final Trigger atComandedStateTrigger;
    private final Trigger beingControlledTrigger;
    private final Trigger possibleShotTrigger;

    private AbleToShootSharedState() {
        this.atComandedStateTrigger = new Trigger(this::getAtTarget);
        this.beingControlledTrigger = new Trigger(this::isBeingControlled);
        this.possibleShotTrigger = new Trigger(this::isPossibleShot);
    }

    public static AbleToShootSharedState getInstance() {
        if (instance == null) {
            instance = new AbleToShootSharedState();
        }
        return instance;
    }

    /**
     * Sets the internal canShoot state. This method should be called externally to update whether
     * the shooter is ready to fire.
     *
     * @param newState The new boolean state for canShoot.
     */
    public void setAtTarget(boolean newState) {
        Log.log("STATUS/CAN SHOOT", newState);
        this.atTarget = newState;
    }

    public void setPossibleShot(boolean newState) {
        Log.log("STATUS/POSSIBLE SHOT", newState);
        this.possibleShot = newState;
    }

    public void setBeingControlled(boolean newState) {
        Log.log("Subsystems/Shooter/BeingControlled", newState);
        this.beingControlled = newState;
    }

    public boolean isBeingControlled() {
        return this.beingControlled;
    }

    public boolean isPossibleShot() {
        return this.possibleShot;
    }

    /**
     * Returns true if the shooter is currently able to shoot. This state is updated externally via
     * setCanShoot().
     *
     * @return boolean indicating if the shooter is ready to shoot.
     */
    public boolean getAtTarget() {
        return this.atTarget;
    }

    /**
     * Provides a Trigger that is active when the shooter is able to shoot. This can be used to bind
     * commands to the "atCommandedState" state.
     *
     * @return a Trigger for the atCommandedState state.
     */
    public Trigger atCommandedStateTrigger() {
        return atComandedStateTrigger;
    }

    public Trigger shotPosible() {
        return possibleShotTrigger;
    }

    public Trigger beingControlledTrigger() {
        return beingControlledTrigger;
    }
}
