/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * Handles autonomous and teleop when the Auto/Manual switch is set to Auto Mode
 * This is the normal setting for competition
 */
public class AutoMode {

    Electronics elec;
    DriverStation dS;
    Action act;
    AutoState autoState;
    RobotLog rLog;
    Location location;
    Side side;
    Holding holding;
    Height height;
    Distance distance;
    boolean isSandStorm = false;
    double turretAngleTarget = -999;
    static final double SHIFTARMSPEED = 0.3;
    Timer matchTimer = new Timer();
    Timer currentTimer = new Timer();

    public enum AutoState {
        ORIGIN, GRABHATCH, GRABBALL, PLACEHATCH, PLACEBALL, UNFOLD
    }

    public enum Location {
        ROCKET, CARGOSHIP
    }

    public enum Side {
        LEFT, RIGHT
    }

    public enum Holding {
        NOTHING, CARGO, HATCH
    }

    public enum Distance {
        FAR, CLOSE
    }

    public enum Height {
        BOTTOM, MIDDLE, TOP
    }

    AutoMode(Electronics electronics, DriverStation driverStation, Action action, RobotLog _log) {
        elec = electronics;
        dS = driverStation;
        act = action;
        rLog = _log;
    }

    void autoInit(boolean _isSandStorm) {
        isSandStorm = _isSandStorm;
        if (isSandStorm) {
            rLog.print("SandStorm: Waiting for Button...");
            matchTimer.start();
        } else {
            rLog.print("Auto Mode Entered: Wating for Button...");
        }
        autoState = AutoState.ORIGIN;
        cancelTurretLock();
        currentTimer.start();
    }

    void cancelTurretLock() {
        turretAngleTarget = -999;
    }

    void waitForButton() {
        if (!dS.getShiftPressed()) {
            if (dS.getDrivingPositionPressed()) {
                autoState = AutoState.UNFOLD;
                act.resetAction();
                cancelTurretLock();
                rLog.print("Button: Driving Position (unfold)");
            } else if (dS.getLFBRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                distance = Distance.FAR;
                height = Height.BOTTOM;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to LFBRocket, sandstorm = " + isSandStorm);
            } else if (dS.getLFMRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                distance = Distance.FAR;
                height = Height.MIDDLE;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to LFMRocket, sandstorm = " + isSandStorm);
            } else if (dS.getLFTRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                distance = Distance.FAR;
                height = Height.TOP;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to LFTRocket, sandstorm = " + isSandStorm);
            } else if (dS.getLCBRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                distance = Distance.CLOSE;
                height = Height.BOTTOM;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to LCBRocket, sandstorm = " + isSandStorm);
            } else if (dS.getLCMRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                distance = Distance.CLOSE;
                height = Height.MIDDLE;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to LCMRocket, sandstorm = " + isSandStorm);
            } else if (dS.getLCTRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                distance = Distance.CLOSE;
                height = Height.TOP;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to LCTRocket, sandstorm = " + isSandStorm);
            } else if (dS.getRFBRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                distance = Distance.FAR;
                height = Height.BOTTOM;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to RFBRocket, sandstorm = " + isSandStorm);
            } else if (dS.getRFMRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                distance = Distance.FAR;
                height = Height.MIDDLE;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to RFMRocket, sandstorm = " + isSandStorm);
            } else if (dS.getRFTRocketPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                distance = Distance.FAR;
                height = Height.TOP;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to RFTRocket, sandstorm = " + isSandStorm);
            } else if (dS.getRCBRocketPressed()) {
                autoState = AutoState.PLACEHATCH;
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                distance = Distance.CLOSE;
                height = Height.BOTTOM;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to RCBRocket, sandstorm = " + isSandStorm);
            } else if (dS.getRCMRocketPressed()) {
                autoState = AutoState.PLACEHATCH;
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                distance = Distance.CLOSE;
                height = Height.MIDDLE;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to RCMRocket, sandstorm = " + isSandStorm);
            } else if (dS.getRCTRocketPressed()) {
                autoState = AutoState.PLACEHATCH;
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                distance = Distance.CLOSE;
                height = Height.TOP;
                location = Location.ROCKET;
                rLog.print("Button: Delivering to RCTRocket, sandstorm = " + isSandStorm);
            } else if (dS.getLCargoPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                location = Location.CARGOSHIP;
                rLog.print("Button: Delievering to LCargo");
            } else if (dS.getRCargoPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                location = Location.CARGOSHIP;
                rLog.print("Button: Delievering to RCargo");
            } else if (dS.getFCargoLPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                location = Location.CARGOSHIP;
                rLog.print("Button: Delievering to FLCargo");
            } else if (dS.getFCargoRPressed()) {
                setStateToPlaceHatchCargo();
                act.resetAction();
                cancelTurretLock();
                side = Side.RIGHT;
                location = Location.CARGOSHIP;
                rLog.print("Button: Delievering to FRCargo");
            } else if (dS.getgrabHatchPressed()) {
                autoState = AutoState.GRABHATCH;
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                rLog.print("Button: Loading hatch");
            } else if (dS.getGrabBallPressed()) {
                autoState = AutoState.GRABBALL;
                act.resetAction();
                cancelTurretLock();
                side = Side.LEFT;
                rLog.print("Button: Loading cargo");
            }
        }
    }

    void setStateToPlaceHatchCargo() {
        if (elec.getGrabberPosition() == 1 || elec.getGrabberPosition() == 0) {
            holding = Holding.NOTHING;
            autoState = AutoState.PLACEHATCH;
            rLog.print("Far Button: Grabber Default = Place Hatch");
        } else if (elec.getGrabberPosition() > 0.5) {
            holding = Holding.CARGO;
            autoState = AutoState.PLACEBALL;
            rLog.print("Far Button: Grabber = Place Cargp");
        } else if (elec.getGrabberPosition() <= 0.5) {
            holding = Holding.HATCH;
            autoState = AutoState.PLACEHATCH;
            rLog.print("Far Button: Grabber = Place Hatch");
        }
    }

    void doPeriodic() {
        if (dS.getCancelPressed() && autoState != AutoState.ORIGIN) {
            autoState = AutoState.ORIGIN;
            cancelTurretLock();
            rLog.print("Button: Canceled");
            rLog.print("Waiting for Button...");
        }

        if (dS.getCancelPressed() && turretAngleTarget != -999) {
            rLog.print("Button: Canceling turret lock");
            cancelTurretLock();
        }

        if (elec.getShoulderAmps() <= Electronics.MAXSHOULDERAMPS && elec.getTurretAmps() <= Electronics.MAXTURRETAMPS
                && elec.getElbowAmps() <= Electronics.MAXELBOWAMPS && elec.getWristAmps() <= Electronics.MAXWRISTAMPS) {
            currentTimer.start();
        } else if (currentTimer.get() > 3) {
            autoState = AutoState.ORIGIN;
            rLog.print("Motor current too high, current action canceled");
            rLog.print("Turret Max: " + elec.totalTurretAmps + " Shoulder Max: " + elec.totalShoulderAmps
                    + " Elbow Max: " + elec.totalElbowAmps + " Wrist Max: " + elec.totalWristAmps);
            rLog.print("Turret: " + elec.getTurretAmps() + " Shoulder: " + elec.getShoulderAmps() + " Elbow: "
                    + elec.getElbowAmps() + " Wrist: " + elec.getWristAmps());
        }

        switch (autoState) {
        case ORIGIN:
            waitForButton();

            if (matchTimer.get() > 5) {
                isSandStorm = false;
            }

            act.allowDriving();

            if (dS.getShiftPressed()) {
                if (dS.getRCBRocketPressed() || dS.getRCMRocketPressed() || dS.getRCTRocketPressed()) {
                    turretAngleTarget = 30;
                    if (!dS.getRCBRocketPressed()) {
                        act.setTargetArm(5, 34, SHIFTARMSPEED);
                    } else if (dS.getRCMRocketPressed()) {
                        act.setTargetArm(3, 36, SHIFTARMSPEED);
                    } else {
                        act.setTargetArm(5, 48, SHIFTARMSPEED);
                    }
                    rLog.print("Button: Shift-near rocket right");
                } else if (dS.getRFBRocketPressed() || dS.getRFMRocketPressed() || dS.getRFTRocketPressed()) {
                    if (elec.getGrabberPosition() > 0.5) {
                        turretAngleTarget = 90;
                        if (dS.getRFBRocketPressed()) {
                            act.setTargetArm(Action.drivingPositionX, Action.drivingPositionY, SHIFTARMSPEED);
                        } else if (dS.getRFMRocketPressed()) {
                            act.setTargetArm(5, 40, SHIFTARMSPEED);
                        } else {
                            act.setTargetArm(3, 48, SHIFTARMSPEED);
                        }
                    } else {
                        turretAngleTarget = 150;
                        if (dS.getRCBRocketPressed()) {
                            act.setTargetArm(5, 34, SHIFTARMSPEED);
                        } else if (dS.getRCMRocketPressed()) {
                            act.setTargetArm(3, 36, SHIFTARMSPEED);
                        } else {
                            act.setTargetArm(5, 48, SHIFTARMSPEED);
                        }
                    }
                    rLog.print("Button: Shift-far rocket right");
                } else if (dS.getLCBRocketPressed() || dS.getLCMRocketPressed() || dS.getLCTRocketPressed()) {
                    turretAngleTarget = -30;
                    if (!dS.getLCBRocketPressed()) {
                        act.setTargetArm(5, 34, SHIFTARMSPEED);
                    } else if (dS.getLCMRocketPressed()) {
                        act.setTargetArm(3, 36, SHIFTARMSPEED);
                    } else {
                        act.setTargetArm(5, 48, SHIFTARMSPEED);
                    }
                    rLog.print("Button: Shift-near rocket left");
                } else if (dS.getLFBRocketPressed() || dS.getLFMRocketPressed() || dS.getLFTRocketPressed()) {
                    if (elec.getGrabberPosition() > 0.5) {
                        turretAngleTarget = -90;
                        if (dS.getLFBRocketPressed()) {
                            act.setTargetArm(Action.drivingPositionX, Action.drivingPositionY, SHIFTARMSPEED);
                        } else if (dS.getLFMRocketPressed()) {
                            act.setTargetArm(5, 40, SHIFTARMSPEED);
                        } else {
                            act.setTargetArm(3, 48, SHIFTARMSPEED);
                        }
                    } else {
                        turretAngleTarget = -150;
                        if (dS.getLCBRocketPressed()) {
                            act.setTargetArm(5, 34, SHIFTARMSPEED);
                        } else if (dS.getLCMRocketPressed()) {
                            act.setTargetArm(3, 36, SHIFTARMSPEED);
                        } else {
                            act.setTargetArm(5, 48, SHIFTARMSPEED);
                        }
                    }
                    rLog.print("Button: Shift-far rocket left");
                } else if (dS.getgrabHatchPressed()) {
                    turretAngleTarget = 180;
                    act.setTargetArm(Action.drivingPositionX, Action.drivingPositionY, SHIFTARMSPEED);
                    rLog.print("Button: Shift-grab hatch");
                } else if (dS.getGrabBallPressed()) {
                    turretAngleTarget = 180;
                    act.setTargetArm(Action.armBackX, Action.GRABCARGOHEIGHT, SHIFTARMSPEED);
                    rLog.print("Button: Shift-grab cargo");
                } else {
                    rLog.print("Shift-nothing");
                }
            }

            if (dS.getTurretM() > 0.5 && dS.getTurretPressed()) {
                act.turnFieldTurret(dS.getTurret());
            } else if (dS.getTurretTurnManual() > 0.2 && dS.getTurretPressed()) {
                double turretSpeed = (dS.getTurretTurnManual() - 0.2) * 0.08;
                elec.setTurretMotor(turretSpeed);
            } else if (dS.getTurretTurnManual() < -0.2 && dS.getTurretPressed()) {
                double turretSpeed = (dS.getTurretTurnManual() + 0.2) * 0.08;
                elec.setTurretMotor(turretSpeed);
            } else if (dS.armTightPressed()) {
                act.turnFieldTurretOptimal(45);
            } else if (turretAngleTarget == -999) {
                elec.setTurretMotor(0);
            } else {
                act.turnFieldTurretOptimal(turretAngleTarget);
            }

            if (dS.armTightPressed()) {
                act.turnShoulder(200);
                elec.setElbowMotor(-0.09);
            } else {
                elec.setShoulderMotor(0);
                elec.setElbowMotor(0);
            }

            if (dS.wristSetUpPressed()) {
                act.setUpWrist();
            } else if (dS.armTightPressed()) {
                act.turnWrist(30);
            } else {
                elec.setWristMotor(0);
            }

            if (dS.moveGrabberPressed()) {
                elec.setGrabberMotor(dS.actuatorValue());
            }

            break;

        case UNFOLD:
            act.allowDriving();
            if (act.unFoldArm()) {
                rLog.print("Done unfolding");
                autoState = AutoState.ORIGIN;
            }
            break;

        case GRABHATCH:
            if (act.grabHatch()) {
                rLog.print("Hatch Grabbed");
                autoState = AutoState.ORIGIN;
            }
            break;

        case GRABBALL:
            if (act.grabCargo()) {
                rLog.print("Cargo Grabbed");
                autoState = AutoState.ORIGIN;
            }
            break;

        case PLACEHATCH:
            switch (location) {

            case ROCKET:
                switch (distance) {
                case CLOSE:
                    if (isSandStorm) {
                        if (act.sandStormNearRocket(height, side)) {
                            rLog.print("HatchPlaced - waiting for button");
                            autoState = AutoState.ORIGIN;
                        }
                    } else if (act.placeHatchRockeClose(height, side)) {
                        rLog.print("HatchPlaced - waiting for button");
                        autoState = AutoState.ORIGIN;
                    }
                    break;

                case FAR:
                    if (isSandStorm) {
                        if (act.sandStormFarRocket(height, side)) {
                            rLog.print("HatchPlaced - waiting for button");
                            autoState = AutoState.ORIGIN;
                        }
                    } else if (act.placeHatchRocketFar(height, side)) {
                        rLog.print("HatchPlaced - waiting for button");
                        autoState = AutoState.ORIGIN;
                    }
                    break;
                }
                break;

            case CARGOSHIP:
                switch (side) {

                case LEFT:
                    break;

                case RIGHT:
                    break;
                }
                break;
            }
            break;

        case PLACEBALL:
            switch (location) {

            case ROCKET:
                if (act.placeBallRocket(height, side)) {
                    rLog.print("Done PlaceCargo - waiting for button");
                    autoState = AutoState.ORIGIN;
                }
                break;

            case CARGOSHIP:
                switch (side) {

                case LEFT:
                    break;

                case RIGHT:
                    break;
                }
                break;
            }
            break;
        }

    }
}