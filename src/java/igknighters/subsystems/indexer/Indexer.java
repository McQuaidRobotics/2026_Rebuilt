package igknighters.subsystems.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.indexer.launcherRollers.*;
import igknighters.subsystems.indexer.launcherRollers.ExitRollers;
import igknighters.subsystems.indexer.spindexer.*;
import igknighters.subsystems.indexer.spindexer.Spindexer;
import igknighters.subsystems.indexer.spindexer.SpindexerSim;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    public enum Mode {
        IDLE,
        DISPENSE,
        UNJAM
    }

    private Mode currentMode = Mode.IDLE;
    private Spindexer spindexer;
    private ExitRollers exitRollers;
    private double goalSpindexerRPM = 0.0;
    private double goalExitRollerRPM = 0.0;
    private final IndexerVisualizer visualizer = new IndexerVisualizer();
    private final Timer wiggleTimer = new Timer();

    public Indexer() {
        if (Robot.isReal()) {
            spindexer = new SpindexerReal();
            exitRollers = new ExitRollersReal();

        } else {
            spindexer = new SpindexerSim();
            exitRollers = new ExitRollersSim();
        }
        wiggleTimer.start();
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    public Mode getMode() {
        return currentMode;
    }

    public void setRPM(double RPM) {
        goalSpindexerRPM = RPM;
        spindexer.goToRPM(RPM);
    }

    public double getSpindexerRPM() {
        return spindexer.getRPM();
    }

    public double getExitRollerRPM() {
        return exitRollers.getSpeedRPM();
    }

    public void goToState(IndexerState state) {
        goalSpindexerRPM = state.getSpindexerRPM();
        goalExitRollerRPM = state.getExitRollerRPM();
        spindexer.goToRPM(goalSpindexerRPM);
        exitRollers.setSpeedRPM(goalExitRollerRPM);
    }

    public void stop() {
        spindexer.stop();
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case DISPENSE:
                // EXIT ROLLERS + INDEXER ROLLERS forward
                goToState(IndexerState.DISPENSE_BALL);
                break;
            case UNJAM:
                // ALL BACKWARDS
                goalSpindexerRPM = -Robot.consts.indexer().kSpindexer().MAX_SPEED_RPM();
                goalExitRollerRPM = -Robot.consts.indexer().kExitRollers().MAX_SPEED_RPM();
                spindexer.goToRPM(goalSpindexerRPM);
                exitRollers.setSpeedRPM(goalExitRollerRPM);
                break;
            case IDLE:
            default:
                // back and forth wiggle
                double wiggleRPM =
                        100.0 * Math.sin(wiggleTimer.get() * 2 * Math.PI * 2.0); // 2Hz wiggle
                goalSpindexerRPM = wiggleRPM;
                goalExitRollerRPM = 0.0;
                spindexer.goToRPM(goalSpindexerRPM);
                exitRollers.setSpeedRPM(goalExitRollerRPM);
                break;
        }

        spindexer.periodic();
        exitRollers.periodic();
        if (Robot.isRobotTest()) {
            Logger.recordOutput("ROBOT/TEST/INDEXER/GOAL_SPINDEXER_RPM", goalSpindexerRPM);
            Logger.recordOutput("ROBOT/TEST/INDEXER/GOAL_EXIT_ROLLER_RPM", goalExitRollerRPM);
            Logger.recordOutput("ROBOT/TEST/INDEXER/CURRENT_SPINDEXER_RPM", spindexer.getRPM());
            Logger.recordOutput(
                    "ROBOT/TEST/INDEXER/CURRENT_EXIT_ROLLER_RPM", exitRollers.getSpeedRPM());
            Logger.recordOutput("ROBOT/TEST/INDEXER/MODE", currentMode.name());
        }
        visualizer.update(spindexer.getRPM(), exitRollers.getSpeedRPM());
    }
}
