/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * This is used to define methods that can be used in autonomous behavior
 */
public class Action {
    private Electronics e;
    private double[] driveEncoderZeros;
    private Timer actionTimer = new Timer();
    private double calcDistTraveled = 0;

    private static final double TARGET_RANGE = 1;

    Action(Electronics elec) {
        e = elec;
        actionTimer.start();
    }

    public void actionReset() {
        driveEncoderZeros = e.getAllDriveEncoders();
        calcDistTraveled = 0;
        actionTimer.reset();
        actionTimer.start();
    }

    public double getActTime() {
        return actionTimer.get();
    }

    public boolean driveStraightNoFacing(double travelAngle, double travelInchesPerSecond, double maxTravelAcceleration,
            double distance, double endingInchesPerSecond) {
        double distTraveled = (e.getAllDriveEncoders()[0] - driveEncoderZeros[0]);
        double distYetToGo = distance - distTraveled;
        if (distTraveled > distance - TARGET_RANGE) {
            e.assignRobotMotionField(travelAngle, endingInchesPerSecond, 0);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double upInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            double dwnInPerSec = endingInchesPerSecond + (distYetToGo * 2);
            double calcInPerSec = Math.min(upInPerSec, dwnInPerSec);
            e.assignRobotMotionField(travelAngle, calcInPerSec, 0);
        }
        return false;
    }

    public boolean driveCurveWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double degreesPerSecond) {
        boolean goalReached = false;
        double goalHeading = startingAngle + (getActTime() * degreesPerSecond);
        if (degreesPerSecond > 0) {
            goalReached = targetAngle - goalHeading < 0.5;
        } else {
            goalReached = targetAngle - goalHeading > -0.5;
        }
        if (goalReached) {
            e.assignRobotMotionAndHeadingField(targetAngle, travelInchesPerSecond, facingAngle);
            return true;
        } else {
            goalHeading = startingAngle + (getActTime() * degreesPerSecond);
            double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            e.assignRobotMotionAndHeadingField(goalHeading, calcInPerSec, facingAngle);
        }
        return false;
    }

    public boolean driveStraightWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration, double distance, double endingInchesPerSecond) {
        calcDistTraveled += e.getCalculatedTravelSinceLastCommand();
        double distYetToGo = distance - calcDistTraveled;
        if (calcDistTraveled > distance - TARGET_RANGE) {
            e.assignRobotMotionAndHeadingField(travelAngle, endingInchesPerSecond, facingAngle);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double upInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            double dwnInPerSec = endingInchesPerSecond + (distYetToGo * 2);
            double calcInPerSec = Math.min(upInPerSec, dwnInPerSec);
            e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
        }
        return false;
    }

    public boolean driveCurveNoFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double maxTravelAcceleration, double degreesPerSecond) {
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, degreesPerSecond);

    }

    public boolean driveToVisionTarget(double facingAngle, double targetDistanceInInches, double targetVisionAngle,
            double distanceRange, double horzDistRange) {
        if (Limelight.isTargetFound()) {
            double angleToTarget = Limelight.getTargetHorizontalAngle() + 4.4; // Limelight camera is not straight
            double forwardDistToTarget = Limelight.getTargetDistanceInInches();
            double horzDistToTarget = (forwardDistToTarget * Math.tan((Math.toRadians(angleToTarget))));
            double forwardDistToGo = forwardDistToTarget - targetDistanceInInches;
            double calcDistToGo = Math.sqrt((Math.pow(horzDistToTarget, 2)) + Math.pow(forwardDistToGo, 2));
            double calcHeading = e.getGyro() + (90 - Math.toDegrees(Math.atan2(forwardDistToGo, horzDistToTarget)));
            if (Math.abs(horzDistToTarget) < horzDistRange && Math.abs(forwardDistToGo) < distanceRange) {
                e.stopMotors();
                return true;
            } else {
                double calcVelocity = calcDistToGo * 2.5;
                e.assignRobotMotionAndHeadingField(calcHeading, calcVelocity, facingAngle);
            }
        } else {
            e.stopMotors();
        }
        return false;
    }

    public double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        if (goalInPerSec >= e.getLastTravelVelocityCommand()) {
            calcInPerSec = Math.min(goalInPerSec,
                    e.getLastTravelVelocityCommand() + e.getMaxSpeedChange(maxTravelAcceleration));
        } else {
            calcInPerSec = Math.max(goalInPerSec,
                    e.getLastTravelVelocityCommand() - e.getMaxSpeedChange(maxTravelAcceleration));
        }
        return calcInPerSec;
    }
}
