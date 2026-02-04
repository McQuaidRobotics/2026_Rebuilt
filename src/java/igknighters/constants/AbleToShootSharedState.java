package igknighters.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger; // Keep Trigger import for internal trigger creation

/**
 * Shared state to determine if the shooter is able to shoot.
 * This class acts as a "lockless" resource, accessible by multiple commands/loops simultaneously.
 * Implemented as a singleton, its 'canShoot' state is updated externally.
 */
public class AbleToShootSharedState {
    private static AbleToShootSharedState instance;

    private boolean canShoot = false;
    private final Trigger canShootTrigger;

    private AbleToShootSharedState() {
        this.canShootTrigger = new Trigger(this::getCanShoot);
    }

    public static AbleToShootSharedState getInstance() {
        if (instance == null) {
            instance = new AbleToShootSharedState();
        }
        return instance;
    }

    /**
     * Sets the internal canShoot state. This method should be called externally
     * to update whether the shooter is ready to fire.
     * @param newState The new boolean state for canShoot.
     */
    public void setCanShoot(boolean newState) {
        this.canShoot = newState;
    }

    /**
     * Returns true if the shooter is currently able to shoot.
     * This state is updated externally via setCanShoot().
     * @return boolean indicating if the shooter is ready to shoot.
     */
    public boolean getCanShoot() {
        return this.canShoot;
    }

    /**
     * Provides a Trigger that is active when the shooter is able to shoot.
     * This can be used to bind commands to the "can shoot" state.
     * @return a Trigger for the canShoot state.
     */
    public Trigger canShootTrigger() {
        return canShootTrigger;
    }
}