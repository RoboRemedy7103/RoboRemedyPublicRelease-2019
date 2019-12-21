/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer; // Do not use '*' for this import statement
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Electronics.RobotName;
import edu.wpi.cscore.UsbCamera;

public class Robot extends TimedRobot {
  Electronics elec = new Electronics();
  DriverStation dS = new DriverStation();
  Action act;
  Arm arm = new Arm();
  PiVision vision;
  ManualMode mm;
  AutoMode auto;
  TeleState teleState = TeleState.MANUAL;
  RobotLog rLog = new RobotLog();
  boolean visionWorking = false;
  boolean wasAuto = false;
  Timer autoTimer = new Timer();
  double testVal = -999;

  boolean wasInit = false;
  int count = 0;
  Timer timer = new Timer();

  public enum TeleState {
    MANUAL, AUTO, UNFOLDARM
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    rLog.print("LiveRobot 7103 robotInit");
    vision = new PiVision(rLog);
    act = new Action(elec, arm, vision, dS, rLog);
    mm = new ManualMode(elec, dS, act, arm, rLog);
    auto = new AutoMode(elec, dS, act, rLog);
    elec.setLED(false);
    if (elec.robot == RobotName.LIVEROBOT) {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(160, 120);
    }
    timer.start();
    rLog.print("Done robotInit");
  }

  /* Called periodically for all modes */
  @Override
  public void robotPeriodic() {
    rLog.incrmentCount();
    vision.processData();
    if (vision.isConnected() && !visionWorking) {
      visionWorking = true;
      rLog.print("Raspberry Pi Vision Connected");
    } else if (!vision.isConnected() && visionWorking) {
      visionWorking = false;
      rLog.print("Raspberry Pi Vision - LOST CONNECTION!");
    }
  }

  void updateDashboard() {
    String s;
    if (!vision.foundTargets()) {
      s = "no target found";
    } else if (vision.getDistance() < 27) {
      s = "too close";
    } else if (vision.getDistance() > 45) {
      s = "too far";
    } else {
      s = "";
    }
    SmartDashboard.putBoolean("Pi Connected", vision.isConnected());
    SmartDashboard.putNumber("Pi Angle", round2(vision.getAngle()));
    SmartDashboard.putNumber("Pi Distance", round2(vision.getDistance()));
    SmartDashboard.putNumber("Gyro Angle", round2(elec.getGyroCenteredOnGoal(0)));
    SmartDashboard.putString("Reason", act.getDashboardReason());
    SmartDashboard.putBoolean("DistanceGood",
        vision.foundTargets() && vision.getDistance() > 27 && vision.getDistance() < 45);
    SmartDashboard.putString("distanceReason", s);
    // SmartDashboard.putNumber("Left Current", elec.getLeftAmps());
    // SmartDashboard.putNumber("Right Current", elec.getRightAmps());
    // SmartDashboard.putNumber("Turret Current", elec.getTurretAmps());
    // SmartDashboard.putNumber("Arm1 Current", elec.getShoulderAmps());
    // SmartDashboard.putNumber("Arm2 Current", elec.getElbowAmps());
    // SmartDashboard.putNumber("Wrist Current", elec.getWristAmps());
    // //SmartDashboard.putNumber("Grabber Current", elec.getGrabberAmps());
    // SmartDashboard.putNumber("Throttle", -dS.getThrottle());
    // SmartDashboard.putNumber("Actuator", dS.actuatorValue());
  }

  /* Called once whenever robot is disabled */
  @Override
  public void disabledInit() {
    rLog.print("LiveRobot 7103 disabledInit");
    vision.setMode("Disa");
    wasInit = false;
    elec.setAllMotorsOff();
    // Display max currents from previous auto or teleop mode
    rLog.print("Turret Max: " + elec.totalTurretAmps + " Shoulder Max: " + elec.totalShoulderAmps + " Elbow Max: "
        + elec.totalElbowAmps + " Wrist Max: " + elec.totalWristAmps);
  }

  /* Called periodically while robot is disabled */
  @Override
  public void disabledPeriodic() {
    updateDashboard();
    elec.disableCompressor();
  }

  /* Called once when autonomous is started */
  @Override
  public void autonomousInit() {
    rLog.print("LiveRobot 7103 autonomousInit");
    vision.setMode("Auto");
    elec.resetGyro(0);
    elec.resetTurret();
    elec.resetDrive();
    if ((dS.getManualModePressed())) {
      teleState = TeleState.MANUAL;
      mm.manualInit();
    } else {
      teleState = TeleState.AUTO;
      auto.autoInit(true);
    }
    act.resetAction();
    elec.setAllMotorsOff();
    wasInit = true;
    rLog.print("Done autonomous Init");
  }

  /* Called periodically during autonomous */
  @Override
  public void autonomousPeriodic() {
    updateDashboard();
    doPeriodic();
    wasAuto = true;
    autoTimer.start();
  }

  /* Called once when operator control is started */
  @Override
  public void teleopInit() {
    rLog.print("teleopInit - wasAuto:" + wasAuto + " timer:" + autoTimer.get());
    vision.setMode("Tele");
    if (!wasAuto || autoTimer.get() > 3) {
      this.autonomousInit();

      if ((dS.getManualModePressed())) {
        teleState = TeleState.MANUAL;
        mm.manualInit();
      } else {
        teleState = TeleState.AUTO;
      }
      act.resetAction();
    }
  }

  /* Called periodically during operator control */
  @Override
  public void teleopPeriodic() {
    updateDashboard();
    doPeriodic();
  }

  void doPeriodic() {
    switch (teleState) {

    case AUTO:
      if (dS.getManualModePressed()) {
        rLog.print("Mode: Changing to Manual Mode");
        act.resetAction();
        mm.manualInit();
        teleState = TeleState.MANUAL;
      } else {
        if (act.actionState == Action.ActionState.MANUALAIM) {
          elec.setLED(false);
        } else {
          elec.setLED(true);
        }
        auto.doPeriodic();
      }
      break;

    case MANUAL:
      if (!dS.getManualModePressed()) {
        rLog.print("Mode: Changing to Auto Mode");
        act.resetAction();
        auto.autoInit(false);
        teleState = TeleState.AUTO;
      } else {
        elec.setLED(false);
        mm.doPeriodic();
      }
      break;

    case UNFOLDARM:
      elec.setLED(false);
      if (act.unFoldArm()) {
        teleState = TeleState.MANUAL;
        mm.manualInit();
        act.resetAction();
      }
      break;
    }

    if (elec.getTurretAmps() > elec.totalTurretAmps) {
      elec.totalTurretAmps = elec.getTurretAmps();
    }
    if (elec.getShoulderAmps() > elec.totalShoulderAmps) {
      elec.totalShoulderAmps = elec.getShoulderAmps();
    }
    if (elec.getElbowAmps() > elec.totalElbowAmps) {
      elec.totalElbowAmps = elec.getElbowAmps();
    }
    if (elec.getWristAmps() > elec.totalWristAmps) {
      elec.totalWristAmps = elec.getWristAmps();
    }

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

  public void testInit() {
    rLog.print("LiveRobot 7103 testInit");
    vision.setMode("Disa");
    elec.resetGyro(0);
    elec.resetTurret();
    elec.resetDrive();
    act.resetAction();
    elec.setAllMotorsOff();
  }

  // Round double to 2 decimal places
  double round2(double val) {
    return Math.round((val * 100.0)) / 100.0;
  }

  /* Called periodically during test mode */
  @Override
  public void testPeriodic() {
    if (dS.test1Pressed()) {
      elec.driveStraight(dS.getThrottleFromTo(0.01, 3.0));
      System.out.println("Foward:" + dS.getThrottleFromTo(0.01, 3.0));
    } else if (dS.test2Pressed()) {
      elec.driveStraight(dS.getThrottleFromTo(-0.01, -3.0));
      System.out.println("Backward:" + dS.getThrottleFromTo(-0.01, -3.0));
    } else {
      elec.driveStraight(0);
    }
  }
}
