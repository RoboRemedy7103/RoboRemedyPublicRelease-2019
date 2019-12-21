/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Controls all the joysticks, buttons, and the dashboard
 */
public class DriverStation {

    Joystick mainJoystick = new Joystick(0); // Logitech Joystick for driver
    Joystick armJoystick = new Joystick(1); // Logitech Joystick for manual control (rarely used in competition)
    // Home-made button panel for the operator uses 2 USB APAC controllers
    Joystick leftAPAC = new Joystick(2); // Left side of the button panel
    Joystick rightAPAC = new Joystick(3); // Right side of the button panel
    JoystickButton drive = new JoystickButton(mainJoystick, 1);
    JoystickButton turnInPlace = new JoystickButton(mainJoystick, 2);
    JoystickButton slowDrive = new JoystickButton(mainJoystick, 3);
    JoystickButton startCompressor = new JoystickButton(mainJoystick, 4);
    JoystickButton solenoidForward = new JoystickButton(mainJoystick, 5);
    JoystickButton solenoidReverse = new JoystickButton(mainJoystick, 6);
    JoystickButton turret = new JoystickButton(armJoystick, 1);
    JoystickButton elbow = new JoystickButton(armJoystick, 3);
    JoystickButton wrist = new JoystickButton(armJoystick, 4);
    JoystickButton shoulder = new JoystickButton(armJoystick, 5);
    JoystickButton moveGrabber = new JoystickButton(armJoystick, 6);
    JoystickButton light = new JoystickButton(armJoystick, 7);
    JoystickButton wristSetUp = new JoystickButton(armJoystick, 9);
    JoystickButton test1 = new JoystickButton(armJoystick, 11);
    JoystickButton test2 = new JoystickButton(armJoystick, 12);
    JoystickButton lFBRocket = new JoystickButton(leftAPAC, 6);
    JoystickButton lFMRocket = new JoystickButton(leftAPAC, 5);
    JoystickButton lFTRocket = new JoystickButton(leftAPAC, 4);
    JoystickButton lCBRocket = new JoystickButton(leftAPAC, 1);
    JoystickButton lCMRocket = new JoystickButton(leftAPAC, 3);
    JoystickButton lCTRocket = new JoystickButton(leftAPAC, 2);
    JoystickButton rFBRocket = new JoystickButton(rightAPAC, 5);
    JoystickButton rFMRocket = new JoystickButton(rightAPAC, 4);
    JoystickButton rFTRocket = new JoystickButton(rightAPAC, 6);
    JoystickButton rCBRocket = new JoystickButton(rightAPAC, 3);
    JoystickButton rCMRocket = new JoystickButton(rightAPAC, 1);
    JoystickButton rCTRocket = new JoystickButton(rightAPAC, 2);
    JoystickButton rCargo = new JoystickButton(rightAPAC, 7);
    JoystickButton grabBall = new JoystickButton(leftAPAC, 11);
    JoystickButton grabHatch = new JoystickButton(leftAPAC, 12);
    JoystickButton drvingPosition = new JoystickButton(rightAPAC, 8);
    JoystickButton manualMode = new JoystickButton(rightAPAC, 10);
    JoystickButton manualMode2 = new JoystickButton(armJoystick, 2);
    JoystickButton cancel = new JoystickButton(rightAPAC, 9);
    JoystickButton superManual = new JoystickButton(leftAPAC, 7);
    JoystickButton verticalAdjust = new JoystickButton(leftAPAC, 8);
    JoystickButton horizontalAdjust = new JoystickButton(rightAPAC, 10);
    JoystickButton manualFinish = new JoystickButton(leftAPAC, 10);
    JoystickButton armTight = new JoystickButton(armJoystick, 10);

    DriverStation() {
    }

    boolean armTightPressed() {
        return armTight.get();
    }

    double getMagnitude() {
        double magnitude = mainJoystick.getMagnitude();
        magnitude = Math.min(magnitude, 1);
        return magnitude;
    }

    // Get Joystick Angle
    double getAngle() {
        double angle = mainJoystick.getDirectionDegrees();
        return angle;
    }

    double getTurretTurnManual() {
        double angleZ = armJoystick.getZ();
        return angleZ;
    }

    double getTurret() {
        double angle = armJoystick.getDirectionDegrees();
        if (angle > -90) {
            return angle;
        } else {
            angle = (360 + angle);
            return angle;
        }
    }

    double getTurretM() {
        double magnitude = armJoystick.getMagnitude();
        return magnitude;
    }

    boolean getTurretPressed() {
        return turret.get();
    }

    double getMainXVal() {
        double mainXVal = mainJoystick.getX();
        return mainXVal;
    }

    double getMainYVal() {
        double mainYVal = -mainJoystick.getY();
        return mainYVal;
    }

    double getArmYVal() {
        double armYVal = -armJoystick.getY();
        return armYVal;
    }

    double getThrottle() {
        double throttleVal = -armJoystick.getThrottle();
        return throttleVal;
    }

    double getThrottleFromTo(double low, double high) {
        // Use throttle to return a value between "low" and "high" (for testing)
        double throttleVal = -armJoystick.getThrottle();
        throttleVal = low + (high - low) * (throttleVal + 1.0) / 2.0;
        return throttleVal;
    }

    boolean getTurnInPlacePressed() {
        return turnInPlace.get();
    }

    boolean getSlowDrivePressed() {
        return slowDrive.get();
    }

    boolean getAlignPressed() {
        return mainJoystick.getRawButton(12);
    }

    boolean getLFBRocketPressed() {
        return lFBRocket.get();
    }

    boolean getLFMRocketPressed() {
        return lFMRocket.get();
    }

    boolean getLFTRocketPressed() {
        return lFTRocket.get();
    }

    boolean getLCBRocketPressed() {
        return lCBRocket.get();
    }

    boolean getLCMRocketPressed() {
        return lCMRocket.get();
    }

    boolean getLCTRocketPressed() {
        return lCTRocket.get();
    }

    boolean getRFBRocketPressed() {
        return rFBRocket.get();
    }

    boolean getRFMRocketPressed() {
        return rFMRocket.get();
    }

    boolean getRFTRocketPressed() {
        return rFTRocket.get();
    }

    boolean getRCBRocketPressed() {
        return rCBRocket.get();
    }

    boolean getRCMRocketPressed() {
        return rCMRocket.get();
    }

    boolean getRCTRocketPressed() {
        return rCTRocket.get();
    }

    boolean getLCargoPressed() {
        return leftAPAC.getY() > 0.5;
    }

    boolean getRCargoPressed() {
        return rCargo.get();
    }

    boolean getFCargoLPressed() {
        return leftAPAC.getY() < -0.5;
    }

    boolean getFCargoRPressed() {
        return leftAPAC.getX() < -0.5;
    }

    boolean getGrabBallPressed() {
        return grabBall.get();
    }

    boolean getgrabHatchPressed() {
        return grabHatch.get();
    }

    boolean getDrivingPositionPressed() {
        return drvingPosition.get();
    }

    boolean getManualModePressed() {
        return (manualMode.get() || manualMode2.get() || turret.get());
    }

    double getInOutSpeed() {
        double speed = -armJoystick.getY();
        if (!manualMode2.get()) {
            return 0;
        } else if (speed > 0.3) {
            return speed - 0.3;
        } else if (speed < -0.3) {
            return speed + 0.3;
        } else {
            return 0;
        }
    }

    double getUpDownSpeed() {
        double speed = armJoystick.getX();
        if (!manualMode2.get()) {
            return 0;
        } else if (speed > 0.3) {
            return speed - 0.3;
        } else if (speed < -0.3) {
            return speed + 0.3;
        } else {
            return 0;
        }
    }

    double getWristUpDownValue() {
        return rightAPAC.getY();
    }

    boolean getCancelPressed() {
        return cancel.get();
    }

    boolean getSuperManualPressed() {
        return superManual.get();
    }

    boolean getHorizontalPressed() {
        return horizontalAdjust.get();
    }

    boolean getVerticalPressed() {
        return verticalAdjust.get();
    }

    boolean lightPressed() {
        return light.get();
    }

    boolean shoulderPressed() {
        return shoulder.get();
    }

    boolean elbowPressed() {
        return elbow.get();
    }

    boolean cancelPressed() {
        return cancel.get();
    }

    boolean moveGrabberPressed() {
        return moveGrabber.get();
    }

    double actuatorValue() {
        // getThrottle normally is from -1 to 1
        // we want a value between Action.GRABBERFULLYCLOSED and 1
        return ((-armJoystick.getThrottle() + 1) / 2) * (1 - Action.GRABBERFULLYCLOSED) + Action.GRABBERFULLYCLOSED;
    }

    boolean drivePressed() {
        return drive.get();
    }

    boolean wristPressed() {
        return wrist.get();
    }

    boolean wristSetUpPressed() {
        return wristSetUp.get();
    }

    boolean test1Pressed() {
        return test1.get();
    }

    boolean test2Pressed() {
        return test2.get();
    }

    boolean getStartCompressorPressed() {
        return startCompressor.get();
    }

    boolean getSolenoidForwardPressed() {
        return solenoidForward.get();
    }

    boolean getSolenoidReversePressed() {
        return solenoidReverse.get();
    }

    boolean getManualControlLeft() {
        return rightAPAC.getY() < -0.5;
    }

    boolean getManualControlRight() {
        return rightAPAC.getX() > 0.5;
    }

    boolean getManualControlIn() {
        return rightAPAC.getY() > 0.5;
    }

    boolean getManualControlOut() {
        return rightAPAC.getX() < -0.5;
    }

    boolean getManualControlFinished() {
        return manualFinish.get();
    }

    boolean getShiftPressed() {
        return leftAPAC.getX() > 0.5;
    }

    boolean getManualAimPressed() {
        // returns true if any one of the manual aim buttons are pressed
        return (getManualControlIn() || getManualControlOut() || getManualControlLeft() || getManualControlRight());
    }
}
