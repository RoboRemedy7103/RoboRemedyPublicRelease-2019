/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Handles reading vision values from the Raspberry Pi
 */
public class PiVision {

    RobotLog rLog;

    private NetworkTableInstance tableInstance;
    private NetworkTable table;

    private boolean foundTargets = false;
    private boolean isConnected = false;
    private double lastTime = 0;
    private double distance = 0.0;
    private double angle = 0.0;
    private int count = 0;
    private int lastPiCount = 0;
    private boolean wasMessageDisplayed = false;

    PiVision(RobotLog _log) {
        rLog = _log;
        tableInstance = NetworkTableInstance.getDefault();
        table = tableInstance.getTable("vision");
    }

    void processData() {
        // This should be called once per loop.
        // Place this call in robotPeriodic()
        double time = System.nanoTime() / 1000000000.0;
        double piDistance = table.getEntry("Distance").getDouble(-999);
        double piAngle = table.getEntry("Angle").getDouble(-999);
        int piCount = table.getEntry("Val").getNumber(-999).intValue();
        if (piCount != lastPiCount) {
            lastTime = time;
            wasMessageDisplayed = false;
        }
        if (time - lastTime > 2.0 && !wasMessageDisplayed) {
            // When we lose connection to the Raspberry pi, we don't get any errors
            // So, we have a table entry that constanly gets increased by the pi ("Val")
            // If this value does not change in 2.0 seconds, print an error message and
            // set isConnected to false
            wasMessageDisplayed = true;
            rLog.print("Vision not connected/Val didn't increase.");
        }
        if (piDistance == -999 || piAngle == -999 || piCount == -999 || !tableInstance.isConnected()
                || time - lastTime > 2.0) {
            isConnected = false;
        } else if (piAngle == 180 || piDistance == -2) {
            // If the Raspberry pi does not find any vision targets, it sets
            // piAngle to 180 and piDistance to -2
            isConnected = true;
            foundTargets = false;
            count = piCount;
        } else {
            isConnected = true;
            foundTargets = true;
            distance = piDistance + 1; // pi is 1" in front of center of robot
            angle = piAngle - 3.6; // pi is mounted facing 3.6 degrees to left
            // Due to the way the Raspberry pi camera is mounted on it's circuit board,
            // this angle adjustment value needs to be checked and tweaked often.
            // Next year: get a different camera or make our own camera mount that is more
            // secure.
            count = piCount;
            if (angle > 10) {
                // Raspberry pi angle calculations are off a bit for angles greater than 10
                // degrees
                // from center. This adjustment was done with trial and error
                angle *= 0.9;
            }
        }
        lastPiCount = piCount;
    }

    public boolean isConnected() {
        return isConnected;
    }

    public boolean foundTargets() {
        return foundTargets;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    public int getCount() {
        return count;
    }

    public boolean setMode(String mode) {
        // Possible values for mode:
        // Disa = Disabled
        // Auto = Autonomous
        // Tele = Teleop
        // Send this value to the Raspberry pi for two reasons:
        // 1) The pi prints this value in it's log file
        // 2) The pi takes image snapshots during Auto and Tele modes
        return table.getEntry("Mode").setString(mode);
    }
}
