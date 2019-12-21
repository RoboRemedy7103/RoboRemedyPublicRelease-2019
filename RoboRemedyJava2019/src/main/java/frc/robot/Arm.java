/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

// This class calculates the positions for two of the three arm motors
// Forward Kinematics: calculate the X and Y position of our wrist joint based on the
//                     current angles of our shoulder, and elbow joints
// Inverse Kinematics: calculate the shoulder and elbow joint angles so that our wrist is at a
//                     certain X and Y position
// X = distance from the center of our robot
// Y = distance from the floor

public class Arm {
    private final static double A = 25; // Length (in.) of arm segment between shoulder and elbow
    private final static double B = 30; // Length (in.) of arm segment between elbow and wrist
    private final static double H = 35.25; // Height (in.) of shoulder joint above ground
    private final static double D = 5.5; // Distance (in.) of shoulder joint from center of robot

    public static void main(String[] args) {
        Arm arm = new Arm();
        RobotLog rlog = new RobotLog();

        rlog.print("Testing for arm calculations");

        arm.displayAngles(/* x */ 13, /* y */83.5);
        arm.displayAngles(3, 34);
        arm.displayAngles(3, 36);
        arm.displayAngles(3, 48);
        arm.displayAngles(3, 40);
        arm.displayAngles(5, 48);
        arm.displayAngles(1, 48);
        arm.displayXY(/* shoulder angle */ 227, /* elbow angle */ 65);
        arm.displayXY(197, 46);
        arm.displayAngles(3, 36);
    }

    void displayAngles(double x, double y) {
        // Test inverse kinematics
        double shoulder = this.getShoulderAngleFromXY(x, y);
        double elbow = this.getElbowAngleFromXY(x, y, shoulder);
        System.out.println("X=" + round(x, 3) + " Y=" + round(y, 3) + " Shoulder=" + round(shoulder, 3) + " Elbow="
                + round(elbow, 3));
    }

    void displayXY(double shoulder, double elbow) {
        // Test forward kinematics
        double x = this.getXFromAngle(shoulder, elbow);
        double y = this.getYFromAngle(shoulder, elbow);
        System.out.println("Shoulder=" + round(shoulder, 3) + " Elbow=" + round(elbow, 3) + " X=" + round(x, 3) + " Y="
                + round(y, 3));
    }

    double sq(double number) {
        return number * number;
    }

    public static double round(double value, int places) {
        if (places < 0)
            throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    // Forward Kinematics:

    double getXFromAngle(double shouderAngle, double elbowAngle) {
        elbowAngle = elbowAngle + 180 - shouderAngle;
        shouderAngle = Math.toRadians(shouderAngle);
        elbowAngle = Math.toRadians(elbowAngle);
        double x = A * Math.sin(shouderAngle) - B * Math.sin(shouderAngle + elbowAngle) - D;
        return x;
    }

    double getYFromAngle(double shoulderAngle, double elbowAngle) {
        elbowAngle = elbowAngle + 180 - shoulderAngle;
        shoulderAngle = Math.toRadians(shoulderAngle);
        elbowAngle = Math.toRadians(elbowAngle);
        double y = -A * Math.cos(shoulderAngle) + B * Math.cos(shoulderAngle + elbowAngle) + H;
        return y;
    }

    // Inverse Kinematics:

    double getShoulderAngleFromXY(double x, double y) {
        x += D;
        y -= H;
        double c2 = sq(x) + sq(y);
        double shoulder1 = Math.toDegrees(Math.acos((A * A + c2 - B * B) / (2.0 * A * Math.sqrt(c2))));
        double shoulder2 = Math.toDegrees(Math.atan(-x / y));
        if (shoulder2 <= 0) {
            shoulder2 += 180;
        }
        double shoulderAngle = shoulder1 + shoulder2;
        return shoulderAngle;
    }

    double getElbowAngleFromXY(double x, double y, double shoulderAngle) {
        x += D;
        y -= H;
        double c2 = sq(x) + sq(y);
        double elbowAngle = Math.acos((A * A + B * B - c2) / (2 * A * B));
        elbowAngle = Math.toDegrees(elbowAngle);
        elbowAngle = elbowAngle - 180 + shoulderAngle;
        return elbowAngle;
    }
}
