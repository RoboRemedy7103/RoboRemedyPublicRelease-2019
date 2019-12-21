/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.*;

/**
 * This is used to control one corner of a swerve drive (one drive and one turn
 * motor)
 */
public class SwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    CANEncoder driveEncoder;
    CANEncoder turnEncoder;
    CANPIDController drivePID;
    CANPIDController turnPID;

    static final double DRIVEVELFACTOR = 0.025208;
    static final double DRIVEPOSFACTOR = DRIVEVELFACTOR * 60;
    static final double TURNPOSFACTOR = 20;

    double turnOffset = 0;

    static final double[] percentValue = { 0.03, 0.07, 0.12, 0.20, 0.40, 0.50, 0.75, 1.00 };
    static final double[] velocityValue = { 3.6, 9.45, 16.6, 28.2, 56.1, 70.1, 104.4, 139.0 };

    SwerveModule(int driveID, int turnID) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        drivePID = driveMotor.getPIDController();
        turnPID = turnMotor.getPIDController();

        driveMotor.setInverted(false);
        turnMotor.setInverted(true);
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kCoast);

        driveEncoder.setVelocityConversionFactor(DRIVEVELFACTOR);
        driveEncoder.setPositionConversionFactor(DRIVEPOSFACTOR);
        turnEncoder.setPositionConversionFactor(TURNPOSFACTOR);

        driveMotor.setSmartCurrentLimit(40);
        turnMotor.setSmartCurrentLimit(30);

        // Feedback values below

        // Still need to figure out PID values
        drivePID.setFF(0);
        drivePID.setP(0);
        drivePID.setI(0);
        drivePID.setD(0);
        drivePID.setIZone(0);
        drivePID.setOutputRange(-1.0, 1.0);

        // Still need to get better PID values
        turnPID.setFF(0);
        turnPID.setP(0.04);
        turnPID.setI(0.0001);
        turnPID.setD(0);
        turnPID.setIZone(3.0);
        turnPID.setOutputRange(-0.8, 0.8);
    }

    void setDrivePower(double power) {
        driveMotor.set(power);
    }

    void setTurnPower(double power) {
        turnMotor.set(power);
    }

    void resetDrivePosition() {
        driveEncoder.setPosition(0);
    }

    void setDrivePosition(double position) {
        driveEncoder.setPosition(position);
    }

    void setTurnEncoderPosition(double position) {
        turnEncoder.setPosition(position);
    }

    double getTurnEncoderPosition() {
        return turnEncoder.getPosition();
    }

    double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public static double getMaxVelocity() {
        return velocityValue[velocityValue.length - 1];
    }

    /*
     * 
     * Methods to use when we are using feeback
     * 
     */

    void setDriveSpeed(double velocity) {
        double feedForward = 12.63 * getPercentageOutputFromVelocity(velocity); // conversion to volts
        drivePID.setReference(velocity, ControlType.kVelocity, 0, feedForward);
    }

    void setTurnHeading(double angle) {
        double turnEncoder = getTurnEncoderPosition();
        while (angle < (turnEncoder - 180)) {
            angle = angle + 360;
        }
        while (angle > (turnEncoder + 180)) {
            angle = angle - 360;
        }
        if (Math.abs(angle - turnEncoder) < 0.24) {
            setTurnPower(0);
        } else {
            turnPID.setReference(angle, ControlType.kPosition);
        }
    }

    void setSpeedAndHeading(double velocity, double angle) {
        if (velocity < 0.01) {
            setDriveSpeed(0);
            setTurnPower(0);
        } else {
            setDriveSpeed(velocity);
            setTurnHeading(angle);
        }
    }

    public double getPercentageOutputFromVelocity(double velocity_In) {
        double percent_Out = 0;
        if (velocity_In >= 0) {
            if (velocity_In < 0.01) {
                percent_Out = 0;
            } else if (velocity_In <= velocityValue[0]) {
                percent_Out = percentValue[0];
            } else if (velocity_In >= velocityValue[velocityValue.length - 1]) {
                percent_Out = percentValue[percentValue.length - 1];
            } else {
                boolean found = false;
                for (int v = 1; !found; v++) {
                    if (velocity_In <= velocityValue[v]) {
                        double rise = velocityValue[v] - velocityValue[v - 1];
                        double run = percentValue[v] - percentValue[v - 1];
                        double slope = rise / run;
                        percent_Out = ((velocity_In - velocityValue[v - 1]) / slope) + percentValue[v - 1];
                        found = true;
                    }
                }
            }
        } else {
            percent_Out = (-1 * getPercentageOutputFromVelocity(-1 * velocity_In));
        }
        return percent_Out;
    }

    /*
     * 
     * Methods to use when tuning new modules
     * 
     */

    void setDriveFPID(double F, double P, double I, double D) {
        drivePID.setFF(F);
        drivePID.setP(P);
        drivePID.setI(I);
        drivePID.setD(D);
    }

    void setTurnFPID(double F, double P, double I, double D) {
        turnPID.setFF(F);
        turnPID.setP(P);
        turnPID.setI(I);
        turnPID.setD(D);
    }

    void setDriveOpenLoopRamp(double rate) {
        driveMotor.setOpenLoopRampRate(rate);
    }

    void setTurnOpenLoopRamp(double rate) {
        turnMotor.setOpenLoopRampRate(rate);
    }
}
