package igknighters.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLights;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLights.PoseMode;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.util.log.Log;

/**
 * Operator controller for vision tuning.
 *
 * <p>A = regular, B = pure MT1, X = pure MT2. DPad up/right/down/left toggles LL1/LL2/LL3/LL4
 * (back, right, turret, intake). Start = reset everything.
 */
public class VisionDebugController {
    private final CommandXboxController controller;

    public VisionDebugController(int port) {
        controller = new CommandXboxController(port);
    }

    public void bind(LimeLightVision vision) {
        LimeLights ll = vision.debug();
        controller.a().onTrue(act(ll, () -> ll.setPoseMode(PoseMode.REGULAR)));
        controller.b().onTrue(act(ll, () -> ll.setPoseMode(PoseMode.PURE_MT1)));
        controller.x().onTrue(act(ll, () -> ll.setPoseMode(PoseMode.PURE_MT2)));
        controller.povUp().onTrue(act(ll, () -> ll.toggleCamera(0)));
        controller.povRight().onTrue(act(ll, () -> ll.toggleCamera(1)));
        controller.povDown().onTrue(act(ll, () -> ll.toggleCamera(2)));
        controller.povLeft().onTrue(act(ll, () -> ll.toggleCamera(3)));
        controller.start().onTrue(act(ll, ll::resetDebug));
        log(ll);
    }

    private static Command act(LimeLights ll, Runnable r) {
        return Commands.runOnce(
                        () -> {
                            r.run();
                            log(ll);
                        })
                .ignoringDisable(true);
    }

    // always logged (ignores disableVisionLogs) so you can see what's active while tuning
    private static void log(LimeLights ll) {
        boolean[] off = ll.getCameraDisabled();
        Log.log("ROBOT/Subsystems/Vision/Debug/Mode", ll.getPoseMode().name());
        for (int i = 0; i < off.length; i++) {
            Log.log("ROBOT/Subsystems/Vision/Debug/LL" + (i + 1) + "_Disabled", off[i]);
        }
    }
}
