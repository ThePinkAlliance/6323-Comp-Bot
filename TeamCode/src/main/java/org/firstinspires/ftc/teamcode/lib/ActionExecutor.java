package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayDeque;

public class ActionExecutor {
    private FtcDashboard dashboard;
    private Action currentAction;

    public ActionExecutor() {
        currentAction = null;
    }

    /**
     * Selects the action that will be executed
     * @param action
     */
    public void run(Action action) {
        this.dashboard = FtcDashboard.getInstance();
        this.currentAction = action;
    }

    /**
     * Runs the currently selected action until finished
     */
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        if (currentAction == null) {
            return;
        }

        // Runs the action at the top of the queue
        boolean isFinished = currentAction.run(packet);

        // when finished remove action from queue
        if (isFinished) {
            currentAction = null;
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
