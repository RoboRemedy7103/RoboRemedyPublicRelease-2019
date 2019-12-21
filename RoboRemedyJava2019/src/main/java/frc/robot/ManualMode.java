/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Electronics.RobotName;

/**
 * Handles autonomous and teleop when the Auto/Manual switch is set to Manual
 * Mode or Super-Manual Mode
 */
public class ManualMode {

  Electronics elec;
  DriverStation dS;
  Action act;
  Arm arm;
  RobotLog rLog;
  boolean bUnfolding = false;

  double shoulderAngle;
  double elbowAngle;
  double wristAngle;
  double xPreset;
  double yPreset;
  double wPreset;
  double gPreset;
  boolean allowSliders = true;
  double wRatio = 25;

  ManualMode(Electronics electronics, DriverStation driverStation, Action action, Arm _arm, RobotLog _log) {
    elec = electronics;
    dS = driverStation;
    act = action;
    arm = _arm;
    rLog = _log;
  }

  void manualInit() {
    assignAngle(elec.getShoulderRotation(), elec.getElbowRotation());
    assignWrist(elec.getWristRotation());
  }

  void doPeriodic() {
    if (dS.getDrivingPositionPressed() && elec.getShoulderRotation() > 240) {
      rLog.print("Driving Position Pressed - Unfolding");
      act.resetAction();
      bUnfolding = true;
    }

    if (bUnfolding) {
      if (act.unFoldArm()) {
        manualInit();
        bUnfolding = false;
      }
    } else if (dS.getSuperManualPressed()) {
      manualInit();
      if (dS.shoulderPressed()) {
        double speed = 0.3 * (dS.getThrottle());
        elec.setShoulderMotor(speed);
      } else {
        elec.setShoulderMotor(0);
      }

      if (dS.elbowPressed()) {
        double speed = 0.3 * (dS.getThrottle());
        elec.setElbowMotor(speed);
      } else {
        elec.setElbowMotor(0);
      }

      if (dS.wristPressed()) {
        double speed = 0.2 * (dS.getThrottle());
        elec.setWristMotor(speed);
      } else {
        elec.setWristMotor(0);
      }

      if (dS.moveGrabberPressed()) {
        rLog.print("Super Manual Mode: Setting Grabber: " + dS.actuatorValue());
        elec.setGrabberMotor(dS.actuatorValue());
      }

      rLog.print("SuperManualMode; Shoulder:" + elec.getShoulderRotation() + " Elbow:" + elec.getElbowRotation()
          + " Wrist:" + elec.getWristRotation() + " turret:" + elec.getTurretRotation());
    } else {
      if (dS.getCancelPressed()) {
        assignAngle(elec.getShoulderRotation(), elec.getElbowRotation());
        assignWrist(elec.getWristRotation());
      }

      if (dS.getDrivingPositionPressed()) {
        rLog.print("Driving Position");
        assignPosition(Action.drivingPositionX, Action.drivingPositionY);
        assignWrist(Action.drivingPositionWrist);
      }

      if (dS.getLCBRocketPressed()) {
        rLog.print("Rocket Position");
        assignPosition(9, Action.GRABHATCHHEIGHT);
        assignWrist(90);
      }

      if (dS.getLCMRocketPressed()) {
        rLog.print("hatch 2");
        assignPosition(4, Action.PUTHATCHMIDHEIGHT);
        assignWrist(90);
      }

      if (dS.getLCTRocketPressed()) {
        rLog.print("hatch 3");
        assignPosition(0, Action.PUTHATCHHIGHHEIGHT);
        assignWrist(90);
      }

      if (dS.getLFBRocketPressed()) {
        rLog.print("cargo 1");
        assignPosition(6, Action.PUTCARGOLOWHEIGHT);
        assignWrist(90);
      }

      if (dS.getLFMRocketPressed()) {
        rLog.print("cargo 2");
        assignPosition(0, Action.PUTCARGOMIDHEIGHT);
        assignWrist(90);
      }

      if (dS.getLFTRocketPressed()) {
        rLog.print("cargo 3");
        assignPosition(0, Action.PUTCARGOHIGHHEIGHT);
        assignWrist(90);
      }

      double inOutSpeed = dS.getInOutSpeed();
      if (inOutSpeed > 0) {
        xPreset = Math.min(xPreset + inOutSpeed * 0.20, 28);
      } else if (inOutSpeed < 0) {
        double xMin;
        if (yPreset >= 18) {
          xMin = 3;
        } else {
          xMin = 8;
        }
        xPreset = Math.max(xPreset + inOutSpeed * 0.20, xMin);
      }

      double upDownSpeed = dS.getUpDownSpeed();
      if (upDownSpeed > 0) {
        yPreset = Math.min(yPreset + upDownSpeed * 0.11, 85);
      } else if (upDownSpeed < 0) {
        double yMin;
        if (xPreset >= 8) {
          yMin = 15;
        } else {
          yMin = 19;
        }
        yPreset = Math.max(yPreset + upDownSpeed * 0.11, yMin);
      }

      if (allowSliders) {
        double x = xPreset;
        double y = yPreset;
        rLog.print("X: " + x + " Y: " + y);
        if ((x >= 8 && y >= 15 && x <= 28) || (x > 3 && y >= 19 && x <= 28)) {
          shoulderAngle = arm.getShoulderAngleFromXY(x, y);
          elbowAngle = arm.getElbowAngleFromXY(x, y, shoulderAngle);
        }
        wristAngle = wPreset + dS.getWristUpDownValue() * wRatio;
      }

      act.turnShoulder(shoulderAngle);
      act.turnElbow(elbowAngle);
      act.turnWrist(wristAngle);
    }

    if (dS.drivePressed()) {
      if (dS.getTurnInPlacePressed()) {
        act.turnInPlaceToAngle(dS.getAngle(), false);
      } else if (dS.getMagnitude() > 0.05) {
        act.driveToAngle(dS.getAngle(), (dS.getMagnitude() - 0.05) * 0.45);
      } else {
        elec.driveStraight(0);
      }
    } else {
      elec.driveStraight(0);
    }

    if (dS.getTurretM() > 0.5 && dS.getTurretPressed()) {
      act.turnFieldTurret(dS.getTurret());
    } else if (dS.getTurretTurnManual() > 0.2 && dS.getTurretPressed()) {
      double turretSpeed = (dS.getTurretTurnManual() - 0.2) * 0.08;
      elec.setTurretMotor(turretSpeed);
    } else if (dS.getTurretTurnManual() < -0.2 && dS.getTurretPressed()) {
      double turretSpeed = (dS.getTurretTurnManual() + 0.2) * 0.08;
      elec.setTurretMotor(turretSpeed);
    } else {
      elec.setTurretMotor(0);
    }

    // Switch Gears
    if (elec.robot == RobotName.LIVEROBOT) {
      if (dS.getStartCompressorPressed()) {
        elec.enableCompressor();
      }
      if (dS.getSolenoidForwardPressed()) {
        elec.solenoid.set(Value.kForward);
      } else if (dS.getSolenoidReversePressed()) {
        elec.solenoid.set(Value.kReverse);
      } else {
        elec.solenoid.set(Value.kOff);
      }
    }
  }

  void assignAngle(double shoulder, double elbow) {
    shoulderAngle = shoulder;
    elbowAngle = elbow;
    xPreset = arm.getXFromAngle(shoulder, elbow);
    yPreset = arm.getYFromAngle(shoulder, elbow);
  }

  void assignPosition(double x, double y) {
    shoulderAngle = arm.getShoulderAngleFromXY(x, y);
    elbowAngle = arm.getElbowAngleFromXY(x, y, shoulderAngle);
    xPreset = x;
    yPreset = y;
  }

  void assignWrist(double wrist) {
    wristAngle = wrist;
    wPreset = wrist - dS.getWristUpDownValue() * wRatio;
  }

}
