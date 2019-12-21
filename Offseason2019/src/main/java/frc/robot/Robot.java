/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;

public class Robot extends TimedRobot {

    final String projectName = "SwerveDrive 7103";
    Joystick joystick1 = new Joystick(0);
    Electronics e;
    Action act;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println(projectName + " robotInit");
        e = new Electronics(true);
        act = new Action(e);
        e.setAllTurnEncoders(0);
    }

    /* Called periodically in all modes */
    @Override
    public void robotPeriodic() {
    }

    /* Called once whenever robot is disabled */
    @Override
    public void disabledInit() {
        System.out.println(projectName + " disabledInit");
    }

    /* Called periodically while robot is disabled */
    @Override
    public void disabledPeriodic() {
    }

    /* Called once when autonomous is started */
    @Override
    public void autonomousInit() {
    }

    /* Called periodically during autonomous */
    @Override
    public void autonomousPeriodic() {
    }

    /* Called once when operator control is started */
    @Override
    public void teleopInit() {
        System.out.println(projectName + " teleopInit");
        e.resetGyro();
        e.setAllTurnEncoders(0);
        act.actionReset();
    }

    /* Called periodically during operator control */
    @Override
    public void teleopPeriodic() {
        double joyMag = Math.pow(joystick1.getMagnitude(), 2);
        double turnMag = Math.copySign(Math.pow(joystick1.getZ(), 2), joystick1.getZ());
        if (joystick1.getRawButton(7) == true) {
            double turnSpeed;
            double speed;
            if (joyMag < 0.2) {
                speed = 0;
            } else {
                speed = (joyMag - 0.2) * (60.0 / 0.8);
            }
            if (turnMag < 0.2 && turnMag > -0.2) {
                turnSpeed = 0;
            } else {
                turnSpeed = Math.copySign((Math.abs(turnMag) - 0.2) * (75.0 / 0.8), turnMag);
            }
            e.assignRobotMotionField(joystick1.getDirectionDegrees(), speed, turnSpeed);
        } else {
            e.stopMotors();
            e.allignMotorsForward();
        }
    }

    /* Called once when test mode is started */
    @Override
    public void testInit() {
    }

    /* Called periodically during test mode */
    @Override
    public void testPeriodic() {
    }
}