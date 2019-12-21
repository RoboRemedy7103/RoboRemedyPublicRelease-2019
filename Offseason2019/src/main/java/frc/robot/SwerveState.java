/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.awt.geom.Point2D;

/**
 * '' This is used to do the swerve drive math calculations
 */
public class SwerveState {

    private class SwerveInstance {
        public double magnitude;
        public double angle;
        public double xFromCenter;
        public double yFromCenter;
    }

    SwerveInstance[] instances;

    public static void main(String[] args) {
        SwerveState swerveState = new SwerveState(30, 30);

        swerveState.assignSwerveModulesField(0, 0, 135, 0, 139);
        swerveState.printValues();

    }

    SwerveState(Point2D.Double[] locations) {
        instances = new SwerveInstance[locations.length];
        for (int i = 0; i < locations.length; i++) {
            instances[i] = new SwerveInstance();
            instances[i].xFromCenter = locations[i].getX();
            instances[i].yFromCenter = locations[i].getY();
        }
    }

    SwerveState(double width, double height) {
        this(new Point2D.Double[] { new Point2D.Double(width * -0.5, height * 0.5),
                new Point2D.Double(width * 0.5, height * 0.5), new Point2D.Double(width * 0.5, height * -0.5),
                new Point2D.Double(width * -0.5, height * -0.5) });
    }

    public void assignSwerveModules(double travelAngle, double travelSpeed, double degreesPerSecond,
            double maxMagnitude) {
        double angularVelocity = Math.toRadians(degreesPerSecond);

        double velX = travelSpeed * Math.sin(Math.toRadians(travelAngle));
        double velY = travelSpeed * Math.cos(Math.toRadians(travelAngle));
        double maxAssignedMag = 0;
        for (SwerveInstance i : instances) {
            double omegaX = (angularVelocity * i.yFromCenter);
            double omegaY = (angularVelocity * i.xFromCenter);
            double moduleX = velX + omegaX;
            double moduleY = velY - omegaY;
            i.magnitude = assignMagnitude(moduleX, moduleY);
            i.angle = assignAngle(moduleX, moduleY);
            maxAssignedMag = Math.max(maxAssignedMag, i.magnitude);
        }
        if (maxAssignedMag > maxMagnitude) {
            for (SwerveInstance i : instances) {
                i.magnitude = i.magnitude / maxAssignedMag * maxMagnitude;
            }
        }

    }

    private static double assignAngle(double x, double y) {
        return Math.toDegrees(Math.atan2(x, y));
    }

    private static double assignMagnitude(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    }

    public void assignSwerveModulesField(double travelAngle, double travelSpeed, double degreesPerSecond,
            double gryoValue, double maxMagnitude) {

        double adjTravelAngle = travelAngle - gryoValue;
        assignSwerveModules(adjTravelAngle, travelSpeed, degreesPerSecond, maxMagnitude);
    }

    private void printValues() {
        System.out.println("FL: " + instances[0].angle + "," + instances[0].magnitude + "  FR: " + instances[1].angle
                + "," + instances[1].magnitude + "  BL: " + instances[2].angle + "," + instances[2].magnitude + "  BR: "
                + instances[3].angle + "," + instances[3].magnitude);
    }

    public double getMagnitude(int index) {
        if (index < 0 || index >= instances.length) {
            return 0;
        } else {
            return instances[(index)].magnitude;
        }
    }

    public double getAngle(int index) {
        if (index < 0 || index >= instances.length) {
            return 0;
        } else {
            return instances[index].angle;
        }
    }

    public void lockWheels() {
        double angle = 45;
        double mult = 1;
        for (SwerveInstance i : instances) {
            if ((i.xFromCenter > 0 && i.yFromCenter > 0) || (i.xFromCenter < 0 && i.yFromCenter < 0)) {
                mult = 1;
            } else {
                mult = -1;
            }
            i.angle = mult * angle;
            i.magnitude = 0;
        }
    }
}
