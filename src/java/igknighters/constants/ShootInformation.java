package igknighters.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Robot;
import igknighters.util.log.Log;
import java.util.function.Supplier;

/**
 * Shared state to determine if the shooter is able to shoot. This class acts as a "lockless"
 * resource, accessible by multiple commands/loops simultaneously. Implemented as a singleton, its
 * 'canShoot' state is updated externally.
 */
public class ShootInformation {
    private static ShootInformation instance;

    private boolean atTarget = false;
    private boolean beingControlled = false;
    private boolean possibleShot = false;
    private final Trigger atComandedStateTrigger;
    private final Trigger beingControlledTrigger;
    private final Trigger possibleShotTrigger;
    private final Trigger useOperatorControlLocationTrigger;
    private boolean useOperatorControlLocation = false;
    private Pose3d operatorControlLocation;

    private ShootInformation() {
        this.atComandedStateTrigger = new Trigger(this::getAtTarget);
        this.beingControlledTrigger = new Trigger(this::isBeingControlled);
        this.possibleShotTrigger = new Trigger(this::isPossibleShot);
        this.useOperatorControlLocationTrigger = new Trigger(this::isUsingOperatorControlLocation);
    }

    public static ShootInformation getInstance() {
        if (instance == null) {
            instance = new ShootInformation();
        }
        return instance;
    }

    public void updateOperatorControlLocation(Pose3d newLocation) {
        this.operatorControlLocation = newLocation;
    }

    public Pose3d getOperatorControlLocation() {
        return operatorControlLocation;
    }

    public Pose3d getHubTarget() {
        return Robot.isBlue() ? FieldConstants.HUB.POSE3D_BLUE : FieldConstants.HUB.POSE3D_RED;
    }

    public Pose3d getPassTarget(Supplier<Pose2d> robotPoSupplier) {
        if (Robot.isBlue()) {
            Pose2d robotPose2d = robotPoSupplier.get();
            return robotPose2d.getY() > FieldConstants.Y_FIELD / 2
                    ? FieldConstants.PASS.POSITION_LEFT_BLUE
                    : FieldConstants.PASS.POSITION_RIGHT_BLUE;
        } else {
            Pose2d robotPose2d = robotPoSupplier.get();
            return robotPose2d.getY() > FieldConstants.Y_FIELD / 2
                    ? FieldConstants.PASS.POSITION_LEFT_RED
                    : FieldConstants.PASS.POSITION_RIGHT_RED;
        }
    }

    public boolean shouldPass(Supplier<Pose2d> robotPoseSupplier) {
        if (Robot.isBlue()) {
            return robotPoseSupplier.get().getX() > FieldConstants.ALIANCE_ZONE_BLUE;
        } else {
            return robotPoseSupplier.get().getX() < FieldConstants.ALIANCE_ZONE_RED;
        }
    }

    public Pose3d getTargetPose(Supplier<Pose2d> robotPoseSupplier) {
        if (shouldPass(robotPoseSupplier)) {
            return getPassTarget(robotPoseSupplier);
        } else {
            return getHubTarget();
        }
    }

    public Pose3d getShotLocation(Supplier<Pose2d> robotPose) {
        if (useOperatorControlLocation) {
            return operatorControlLocation;
        }
        if (shouldPass(robotPose)) {
            return getPassTarget(robotPose);
        }
        return getHubTarget();
    }

    public void useOperatorControlLocation(boolean use) {
        this.useOperatorControlLocation = use;
    }

    public boolean isUsingOperatorControlLocation() {
        return useOperatorControlLocation;
    }

    public Trigger useOperatorControlLocationTrigger() {
        return useOperatorControlLocationTrigger;
    }

    /**
     * Sets the internal canShoot state. This method should be called externally to update whether
     * the shooter is ready to fire.
     *
     * @param newState The new boolean state for canShoot.
     */
    public void setAtTarget(boolean newState) {
        Log.log("LOGGING/STATUS/CAN SHOOT", newState);
        this.atTarget = newState;
    }

    public Trigger canShoot() {
        return atComandedStateTrigger.and(beingControlledTrigger).and(possibleShotTrigger);
    }

    public void setPossibleShot(boolean newState) {
        Log.log("LOGGING/STATUS/POSSIBLE SHOT", newState);
        this.possibleShot = newState;
    }

    public void setBeingControlled(boolean newState) {
        Log.log("LOGGING/Subsystems/Shooter/BeingControlled", newState);
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
