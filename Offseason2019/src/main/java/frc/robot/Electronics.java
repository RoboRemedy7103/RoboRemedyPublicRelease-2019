/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.kauailabs.navx.frc.*;

/**
 * This is used to control the motors, gyro, and encoders
 */
public class Electronics {

    private SwerveModule frontLeft = new SwerveModule(1, 11);
    private SwerveModule frontRight = new SwerveModule(2, 12);
    private SwerveModule backRight = new SwerveModule(3, 13);
    private SwerveModule backLeft = new SwerveModule(4, 14);
    private SwerveModule[] swerveDrives;
    private SwerveState state = new SwerveState(13.5, 13.5);
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double lastTravelVelocity = 0;
    private Timer commandTimer = new Timer();
    private double lastCommandTime = 0;

    Electronics(boolean fullSwerve) {
        if (fullSwerve) {
            swerveDrives = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };
        } else {
            swerveDrives = new SwerveModule[] { backLeft };
        }
        commandTimer.start();
    }

    public void stopMotors() {
        lastTravelVelocity = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDrivePower(0);
            m.setTurnPower(0);
        }
    }

    public void allignMotorsForward() {
        lastTravelVelocity = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDrivePower(0);
            m.setTurnHeading(0);
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getGyro() {
        return gyro.getAngle();
    }

    public double getGyroCenteredOnGoal(double goalAngle) {
        double gyroValue = getGyro();

        while (gyroValue < (goalAngle - 180)) {
            gyroValue += 360;
        }
        while (gyroValue > (goalAngle + 180)) {
            gyroValue -= 360;
        }

        return gyroValue;
    }

    private SwerveModule getModule(int module) {
        if (module >= swerveDrives.length || module < 0) {
            return null;
        } else {
            return swerveDrives[module];
        }
    }

    public void setDrivePercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            System.out.println("Not Valid Module ");
        } else {
            swerveDriveModule.setDrivePower(power);
        }
    }

    public void setDriveSpeed(int moduleNumber, double velocity) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            System.out.println("Not Valid Module ");
        } else {
            swerveDriveModule.setDriveSpeed(velocity);
        }
    }

    public void setTurnPercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            System.out.println("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnPower(power);
        }
    }

    public void setTurnHeading(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            System.out.println("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnHeading(angle);
        }
    }

    public void setAllHeadings(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnHeading(i, angle);
        }
    }

    public void setTurnEncoder(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            System.out.println("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnEncoderPosition(angle);
        }
    }

    public void setAllTurnEncoders(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnEncoder(i, angle);
        }
    }

    public double[] getAllTurnEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getTurnEncoderPosition();
        }
        return positions;
    }

    public double[] getAllDriveEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getDriveEncoderPosition();
        }
        return positions;
    }

    public double getDriveVelocity(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            System.out.println("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveVelocity();
        }
    }

    public void assignRobotMotionField(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastCommandTime = commandTimer.get();
        state.assignSwerveModulesField(travelAngle, travelInchesPerSecond, degreesPerSecond, getGyro(),
                SwerveModule.getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(state.getMagnitude(i), state.getAngle(i));
        }
    }

    public void assignRobotMotionAndHeadingField(double travelAngle, double travelInchesPerSecond, double facingAngle) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 2.5), 1);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 2.0;
            if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-120, calcDPS), -25);
            } else {
                degreesPerSecond = Math.max(Math.min(120, calcDPS), 25);
            }
        }
        assignRobotMotionField(travelAngle, travelInchesPerSecond, degreesPerSecond);
    }

    public void lockWheels() {
        lastTravelVelocity = 0;
        lastCommandTime = commandTimer.get();
        state.lockWheels();
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(state.getMagnitude(i), state.getAngle(i));
        }
    }

    public double getLastTravelVelocityCommand() {
        return lastTravelVelocity;
    }

    public double getCalculatedTravelSinceLastCommand() {
        return lastTravelVelocity * (commandTimer.get() - lastCommandTime);
    }

    public double getMaxSpeedChange(double maxTravelAcceleration) {
        return maxTravelAcceleration * (commandTimer.get() - lastCommandTime);
    }
}
