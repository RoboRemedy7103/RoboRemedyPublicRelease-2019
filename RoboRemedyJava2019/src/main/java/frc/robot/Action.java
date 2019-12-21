/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AutoMode;
import frc.robot.AutoMode.Height;
import frc.robot.Electronics.RobotName;

/**
 * This class has all of the actions that can be run on robot
 */
public class Action {

    Electronics elec;
    DriverStation dS;
    Arm arm;
    PiVision vision;
    RobotLog rLog;
    String dashboardReason = "";

    Timer timer = new Timer();

    // Turret values

    private static double MINTURRETGOAL = 1.0;
    private static double LOWTURRETSPEED = 0.057;
    private static double HIGHTURRETSPEED = 0.4;
    private static double DVRATIO = 0.005;
    private static double FASTVISIONANGLE = 3.5;

    // Drive values

    private static double TURNSTEERRATIO = 0.03;
    private static double TURNRATIO = 0.0030; // For in-place turns
    private static double MAXTURNSPEED = 0.4;
    private static double MINTURNSPEED = 0;
    private static double MINTURNGOAL = 1.1;

    // J1 = Joint 1 = Shoulder Values

    private static double MINJ1GOAL = 1;
    private static double LOWJ1SPEEDUP = 0.16;
    private static double HIGHJ1SPEEDUP = 0.54;
    private static double J1RATIOUP = 0.016;
    private static double LOWJ1SPEEDDOWN = 0.085;
    private static double HIGHJ1SPEEDDOWN = 0.34;
    private static double J1RATIODOWN = 0.012;
    private static double LOWJ1UNFOLDSPEED = 0.12;
    private static double HIGHJ1UNFOLDSPEED = 0.38;
    private static double J1UNFOLDRATIO = 0.008;
    private static double J1DRIVETIGHTSPEED = 0.16;

    // J2 = Joint 2 = Elbow Values

    private static double MINJ2GOAL = 1;
    private static double LOWJ2SPEEDUP = 0.1;
    private static double HIGHJ2SPEEDUP = 0.42;
    private static double J2RATIOUP = 0.025;
    private static double LOWJ2SPEEDDOWN = 0.049;
    private static double HIGHJ2SPEEDDOWN = 0.33;
    private static double J2RATIODOWN = 0.012;
    private static double J2DRIVETIGHTSPEED = -0.09;

    // W = Wrist Values

    private static double MINWGOAL = 1;
    private static double LOWWSPEEDUP = 0.09;
    private static double HIGHWSPEEDUP = 0.24;
    private static double WRATIOUP = 0.02;
    private static double LOWWSPEEDDOWN = 0.07;
    private static double HIGHWSPEEDDOWN = 0.24;
    private static double WRATIODOWN = 0.025;

    // Grabber linear acuator values

    public static final double GRABBERHOLDINGHATCH = 0.48111;
    public static final double GRABBERFULLYCLOSED = 0.27111;
    public static final double GRABBERHOLDINGCARGO = 0.53;
    public static final double GRABBERFULLYOPEN = 0.95;

    // Field heights

    public static final double GRABHATCHHEIGHT = 19;
    public static final double PUTHATCHLOWHEIGHT = 21;
    public static final double PUTHATCHMIDHEIGHT = 50; // center = 49
    public static final double PUTHATCHHIGHHEIGHT = 78;
    public static final double GRABCARGOHEIGHT = 34;
    public static final double PUTCARGOLOWHEIGHT = 29; // center = 27.5
    public static final double PUTCARGOMIDHEIGHT = 58; // center = 55.5
    public static final double PUTCARGOHIGHHEIGHT = 85; // center = 83.5

    // Lengths of grabber

    private static final double GRABBERLENGTHCLOSED = 19;
    private static final double GRABBERLENGTHWITHHATCH = 17.5;
    private static final double GRABBERLENGTHOPEN = 18.5;
    private static final double GRABBERLENGTHWITHBALL = 18;
    private static final double GRABBERLENGTHINCLUDINGCARGO = 22;

    public enum UnFoldStage {
        stage1, stage2, stage3
    }

    // Initially we used angles to determine default driving position of our arm
    // Later we converted most of our code to use X,Y for default driving position
    // There is still some code that uses the angles

    public static final double[] drivingPositionAngle = { 204, 38.5, 138 };
    public static final double drivingPositionX = 3;
    public static final double drivingPositionY = 34.6;
    public static final double drivingPositionWrist = 138;

    public static final double GRABCARGOWRIST = 120; // Angle for grabbing cargo from loading station
    public static final double armBackX = 3; // Grabbing/placing wrist X position for manual mode (no vision processing)

    public enum ActionState {
        BEGIN, TURNTURRET, TURRETTOTARGET, TURNINPLACE, DRIVETOTARGET, DRIVE1, DRIVE2, VISIONPROCESS, MOVEARM1,
        MOVEARM2, MOVEARM3, GRABHATCH, GETDISTANCE, OPENGRABBER, CLOSEGRABBER, GRABBALL, PLACEBALL, PLACEHATCH,
        WITHDRAWARM, BACKUP, WAIT, UNFOLD, LINEUP, DRIVESTRAIGHT, DRIVINGPOSITION, WAITFORBUTTON, TURNWRIST, MANUALAIM
    }

    public enum PlaceHatchState {
        BEGIN, TURNTURRET, TURRETTOTARGET, GETDISTANCE, MOVEARMCLOSE, MOVEARMIN, PLACEHATCH, MOVEARMOUT, OPENGRABBER,
        DRIVINGPOSITION, MANUALAIM, PREMANUALAIM
    }

    UnFoldStage unFoldStage = UnFoldStage.stage1;
    ActionState actionState = ActionState.BEGIN;
    PlaceHatchState placeHatchState;
    double piDistance;
    double distance;
    double visionAngle;
    double turretAngle;
    double goalShoulder;
    double goalElbow;
    double goalY;
    double goalX;
    double targetX;
    double targetY;
    double targetSpeed;
    double targetWrist;
    double placedX;
    double placedY;
    double placedWrist;
    int visionCount = 0;
    int initialVisionTurretCount = 0;
    double visionTurretTarget = 0.0;
    boolean foundVisionTurretAngle = false;
    boolean visionTurretFailed = false;
    Timer visionTurretTimer = new Timer();
    boolean reverseMode = false;
    boolean hatchWasPlaced = false;
    boolean cargoWasPlaced = false;

    Action(Electronics electronics, Arm _arm, PiVision _vision, DriverStation _driverStation, RobotLog _log) {
        elec = electronics;
        vision = _vision;
        arm = _arm;
        dS = _driverStation;
        rLog = _log;
        if (elec.robot == RobotName.MRJ) {
            TURNRATIO = 0.004;
            TURNSTEERRATIO = 0.8;
        }
    }

    void resetAction() {
        unFoldStage = UnFoldStage.stage1;
        actionState = ActionState.BEGIN;
        placeHatchState = PlaceHatchState.BEGIN;
        timer.reset();
        timer.start();
    }

    void setDashboardReason(String s) {
        dashboardReason = s;
    }

    String getDashboardReason() {
        return dashboardReason;
    }

    // Round double to 2 decimal places
    double round2(double val) {
        return Math.round((val * 100.0)) / 100.0;
    }

    void driveToAngle(double goalAngle, double speed) {
        // This action is used to drive the robot at a certain speed toward a certain
        // angle
        // Uses gyro to auto-correct if robot is not going toward correct angle
        // Robot Will drive backwards instead of forwards if that means less of a turn
        // Due to the turret, this robot can grab or place in any direction
        double gyroValue = elec.getGyroCenteredOnGoal(goalAngle);
        double initialSpeed = speed;
        double angleDifference = 0;
        double adjustment = 0;
        double leftMultiplier;
        double rightMultiplier;
        double finalLeftSpeed = 0;
        double finalRightSpeed = 0;
        reverseMode = false;

        angleDifference = (gyroValue - goalAngle);

        if (angleDifference > 90) {
            reverseMode = true;
            angleDifference -= 180;
        }

        if (angleDifference < -90) {
            reverseMode = true;
            angleDifference += 180;
        }
        if (reverseMode) {
            goalAngle = goalAngle + 180;
        }

        if (Math.abs(angleDifference) > 60 && speed > 0.1) {
            // If the robot is more than 60 degrees away from desired angle, then
            // don't drive forward, instead just turn in place
            turnInPlaceToAngle(goalAngle, true);
            return;
        }
        adjustment = Math.min(TURNSTEERRATIO * Math.abs(angleDifference), 0.6);

        if (angleDifference > 0) {
            leftMultiplier = 1 - adjustment;
            rightMultiplier = 1;
        } else {
            rightMultiplier = 1 - adjustment;
            leftMultiplier = 1;
        }

        finalLeftSpeed = leftMultiplier * initialSpeed;
        finalRightSpeed = rightMultiplier * initialSpeed;

        if (reverseMode) {
            elec.setLeftAdjSpeed(-finalRightSpeed);
            elec.setRightAdjSpeed(-finalLeftSpeed);
        } else {
            elec.setLeftAdjSpeed(finalLeftSpeed);
            elec.setRightAdjSpeed(finalRightSpeed);
        }
    }

    void armDriveTight() {
        // Drive the arm motors tight to prevent arm damage and prevent the hatch panel
        // from falling off while driving off the platform
        // Used during sandstorm
        if (elec.getShoulderRotation() > 350) {
            elec.setShoulderMotor(J1DRIVETIGHTSPEED);
            elec.setElbowMotor(J2DRIVETIGHTSPEED);
        } else {
            elec.setShoulderMotor(0);
            elec.setElbowMotor(0);
        }
    }

    boolean armToDrivePosition(double speed) {
        // This action places that three arm joints in the standard driving
        // position. The shoulder and elbow joints move at "speed" speed.
        turnWrist(drivingPositionWrist);
        return moveToGoalPosition(drivingPositionX, drivingPositionY, speed);
    }

    boolean armToDrivePosition() {
        // This action places that three arm joints in the standard driving
        // position. The shoulder and elbow joints move at normal speed.
        turnWrist(drivingPositionWrist);
        if (turnShoulder(drivingPositionAngle[0]) & turnElbow(drivingPositionAngle[1])) {
            return true;
        }
        return false;
    }

    boolean armToGoalPosition() {
        // This action moves the shoulder and elbow to the goal position
        // (goalShoulder and goalElbow)
        if (turnShoulder(goalShoulder) & turnElbow(goalElbow)) {
            return true;
        }
        return false;
    }

    boolean armToGoalPositionFast() {
        // This action moves the shoulder and elbow to the goal position
        // (goalShoulder and goalElbow) using a faster speed
        if (turnShoulderFast(goalShoulder) & turnElbowFast(goalElbow)) {
            return true;
        }
        return false;
    }

    boolean turnInPlaceToAngle(double goalAngle, boolean allowReverseMode) {
        // This action turns the robot in place to a specic angle
        // If allowReverseMode is true, the robot can turn either the front or
        // back to that goal angle, whichever is closer
        double gyroValue = elec.getGyroCenteredOnGoal(goalAngle);
        double angleDifference = 0;
        double turnSpeed;
        reverseMode = false;

        angleDifference = (gyroValue - goalAngle);

        if (angleDifference > 90 && allowReverseMode) {
            reverseMode = true;
            angleDifference -= 180;
        }

        if (angleDifference < -90 && allowReverseMode) {
            reverseMode = true;
            angleDifference += 180;
        }

        if (reverseMode) {
            goalAngle = goalAngle + 180;
        }

        if (Math.abs(angleDifference) <= MINTURNGOAL) {
            turnSpeed = 0;
        } else {
            turnSpeed = Math.abs(angleDifference * TURNRATIO);
            turnSpeed = Math.min(turnSpeed, MAXTURNSPEED);
            turnSpeed = Math.max(turnSpeed, MINTURNSPEED);
            if (angleDifference < 0) {
                turnSpeed = -turnSpeed;
            }
        }

        elec.setLeftAdjSpeed(-turnSpeed);
        elec.setRightAdjSpeed(turnSpeed);

        if (Math.abs(angleDifference) <= MINTURNGOAL) {
            elec.driveStraight(0);
            return true;
        }
        return false;
    }

    boolean startFastTurretToTarget() {
        // Preparse for fast turret targeting
        foundVisionTurretAngle = false;
        if (vision.isConnected()) {
            initialVisionTurretCount = vision.getCount();
            visionTurretFailed = false;
            visionTurretTimer.reset();
            visionTurretTimer.start();
            rLog.print("Fast Turret: Start at count=" + initialVisionTurretCount);
            return true;
        } else {
            rLog.print("Fast Turret: Start failed due to no vision connected");
            visionTurretFailed = true;
            return false;
        }
    }

    boolean getFastTurretToTarget(double targetAngle) {
        // Attempt to find the vision target and calculate the distance and angle
        if (visionTurretFailed || foundVisionTurretAngle) {
            return true;
        }
        if (vision.foundTargets() && vision.getCount() > initialVisionTurretCount + 2) {
            double visionAngle = vision.getAngle();
            double visionDistance = vision.getDistance();
            double turretAngle = elec.getTurretFieldCenteredOnGoal(targetAngle);
            double multiplier = Math.cos(Math.toRadians(targetAngle - turretAngle));
            double goalAngle = turretAngle - (visionAngle / multiplier);
            double goalDistance = visionDistance * multiplier;
            rLog.print("Vision: " + visionAngle + " turret:" + turretAngle + " target:" + targetAngle + " cos:"
                    + multiplier + " visionD:" + visionDistance + " distance:" + goalDistance);
            visionTurretTarget = goalAngle;
            distance = goalDistance;
            foundVisionTurretAngle = true;
            rLog.print("Fast Turret: Found Target: a:" + visionTurretTarget + ",d:" + distance + ",c:"
                    + vision.getCount() + " time:" + visionTurretTimer.get());
            return true;
        }
        if (!vision.isConnected()) {
            rLog.print("Fast Turret: Get failed due to no vision connected");
            visionTurretFailed = true;
            return true;
        } else if (visionTurretTimer.get() > 0.75) {
            rLog.print(
                    "Fast Turret: Failed due to time out. count=" + initialVisionTurretCount + "," + vision.getCount());
            visionTurretFailed = true;
            return true;
        }
        return false;
    }

    boolean turretToTarget() {
        // Turn the target so it directly faces the vision target
        double visionAngle = vision.getAngle();
        double speed = 0.0;
        boolean isDone = false;

        if (!vision.isConnected()) {
            rLog.print("Not connected");
        } else if (!vision.foundTargets()) {
            rLog.print("No targets found");
        } else if (Math.abs(visionAngle) < 1.8) {
            isDone = true;
        } else if (visionAngle > 0 && visionAngle < 3.0) {
            if (visionCount % 6 > 2) {
                speed = -LOWTURRETSPEED; // Only powered 50% of time
            }
            visionCount++;
        } else if (visionAngle < 0 && visionAngle > -3.0) {
            if (visionCount % 6 > 2) {
                speed = LOWTURRETSPEED; // Only powered 50% of time
            }
            visionCount++;
        } else if (visionAngle > 0) {
            speed = -LOWTURRETSPEED;
            visionCount = 0;
        } else {
            speed = LOWTURRETSPEED;
            visionCount = 0;
        }
        elec.setTurretMotor(speed);
        return isDone;
    }

    boolean driveToTarget(double goalDistance, double goalAngle) {
        // This action will drive the robot forward, while placing the robot
        // directly in front of the vision target, stopping "goalDisatance" away
        // from the vision target.
        // goalAngle is the actual field angle of the target
        // This code is not actually used because our test robot performed
        // different from our live robot and we did not have time to get our
        // live robot working with this code
        double actualDistance = vision.getDistance();
        double distanceToGo = (actualDistance - goalDistance);
        double visionAngle = vision.getAngle();
        double steeringAngle = elec.getTurretFieldCenteredOnGoal(goalAngle);
        double errorAngle = (goalAngle - (steeringAngle - visionAngle));
        double errorMultiplier;
        double calcSpeed = 0;
        double finalSpeed = 0;

        // this.turretToTarget();
        turnFieldTurret(goalAngle);
        if (!vision.isConnected()) {
            elec.driveStraight(0);
            rLog.print("Not connected");
            return false;
        } else if (!vision.foundTargets()) {
            elec.driveStraight(0);
            rLog.print("No targets found - goalAngle: " + goalAngle);
            return false;
        } else if (distanceToGo < 6 & distanceToGo > -6) {
            elec.driveStraight(0);
            rLog.print("Reached goal, turret fied angle: " + visionAngle);
            if (turretToTarget()) {
                return true;
            }
            return false;
        } else if (distanceToGo < 0) {
            driveToAngle(goalAngle - 180.0, 0.01);
            rLog.print("TOO CLOSE");
            return false;
        } else {
            if (distanceToGo < 48) {
                errorMultiplier = 0; // Math.max(2.0*(distanceToGo/18), 0.5);
            } else {
                errorMultiplier = 1.2;
            }
            calcSpeed = 0.005 * distanceToGo;
            finalSpeed = Math.min(calcSpeed, 0.33);
            // finalSpeed = Math.max(finalSpeed, 0.25); //0.15 was without the plus
            double newSteeringAngle = (goalAngle - errorMultiplier * (errorAngle));
            // newSteeringAngle = Math.max(Math.min(newSteeringAngle, goalAngle + 30),
            // goalAngle - 30);
            // newSteeringAngle = Math.max(Math.min(newSteeringAngle, visionAngle + 30),
            // (visionAngle - 30));
            this.driveToAngle(newSteeringAngle, finalSpeed);
            rLog.print("distToGo: " + distanceToGo + "finalSpeed: " + finalSpeed);
            rLog.print("S Angle: " + steeringAngle + " S' Angle: " + newSteeringAngle + " V Angle: " + visionAngle
                    + " E Angle: " + errorAngle);
            return false;
        }
    }

    boolean driveToTargetStraight(double goalDistance, double goalAngle) {
        // This action will drive the robot forward at a specific angle
        // until vision target is a specific distance away, ending with
        // zero speed. It will not adjust angle for vision target
        double actualDistance = vision.getDistance();
        double distanceToGo = (actualDistance - goalDistance);
        double calcSpeed = 0;
        double finalSpeed = 0;

        // this.turretToTarget();

        if (!vision.isConnected()) {
            elec.driveStraight(0);
            rLog.print("Not connected");
            return false;
        } else if (!vision.foundTargets()) {
            elec.driveStraight(0);
            rLog.print("No targets found - goalAngle: " + goalAngle);
            return false;
        } else if (distanceToGo < 6 & distanceToGo > -6) {
            elec.driveStraight(0);
            rLog.print("Reached goal for loading");
            if (turretToTarget()) {
                rLog.print("And the turret too");
                return true;
            }
            return false;
        } else if (distanceToGo < 0) {
            elec.driveStraight(-0.2);
            rLog.print("TOO CLOSE");
            return false;
        } else {
            calcSpeed = 0.005 * distanceToGo;
            finalSpeed = Math.min(calcSpeed, 0.33);
            this.driveToAngle(goalAngle, finalSpeed);
            return false;
        }
    }

    boolean driveDistanceToAngle(double goalDistance, double goalAngle) {
        // This action will drive the robot forward at a specific angle
        // for a specific distance in inches, ending with zero speed
        double gyroAngle = elec.getGyroCenteredOnGoal(goalAngle);
        boolean reverseMode = false;
        double distanceToGo = (goalDistance - elec.getRobotDisplacement());
        double calcSpeed = 0;
        double finalSpeed = 0;

        if (Math.abs(gyroAngle - goalAngle) > 90) {
            reverseMode = true;
            distanceToGo = (goalDistance + elec.getRobotDisplacement());
        }

        if (distanceToGo < 6 & distanceToGo > -6) {
            elec.driveStraight(0);
            rLog.print("Reached goal for distance");
            return true;
        } else if (distanceToGo < 0) {
            if (reverseMode)
                elec.driveStraight(0.2);
            else
                elec.driveStraight(-0.2);
            rLog.print("TOO CLOSE");
            return false;
        } else {
            calcSpeed = 0.005 * distanceToGo;
            finalSpeed = Math.min(calcSpeed, 0.45);
            this.driveToAngle(goalAngle, finalSpeed);
            return false;
        }
    }

    boolean turnTurret(double goalAngle) {
        // This action will turn the turret toward a specific angle
        // based on the robot point of view
        double currentAngle = elec.getTurretRotation();
        double difference = goalAngle - currentAngle;
        double speed = difference * DVRATIO;
        boolean isDone = false;
        if (speed > 0) {
            speed = Math.max(speed, LOWTURRETSPEED);
            speed = Math.min(speed, HIGHTURRETSPEED);
        }
        if (speed < 0) {
            speed = Math.min(speed, -LOWTURRETSPEED);
            speed = Math.max(speed, -HIGHTURRETSPEED);
        }
        if (Math.abs(difference) <= MINTURRETGOAL) {
            speed = 0;
            isDone = true;
        }
        elec.setTurretMotor(speed);
        return isDone;
    }

    boolean turnFieldTurret(double goalAngle) {
        // This action will turn the turret toward a specific angle
        // based on the field point of view
        // Turn in the direction that is the shortest distance
        double currentAngle = elec.getTurretFieldCenteredOnGoal(goalAngle);
        double difference = goalAngle - currentAngle;
        double actualGoal = elec.getTurretRotation() + difference;

        // if the goalAngle is past the soft limit, this moves the turret the opposite
        // way
        if (actualGoal > Electronics.FORWARDTHRESHOLD) {
            difference = difference - 360;
        }
        if (actualGoal < Electronics.REVERSETHRESHOLD) {
            difference = difference + 360;
        }
        double speed = difference * DVRATIO;
        boolean isDone = false;

        if (speed > 0) {
            speed = Math.max(speed, LOWTURRETSPEED);
            speed = Math.min(speed, HIGHTURRETSPEED);
        }
        if (speed < 0) {
            speed = Math.min(speed, -LOWTURRETSPEED);
            speed = Math.max(speed, -HIGHTURRETSPEED);
        }
        if (Math.abs(difference) <= MINTURRETGOAL) {
            speed = 0;
            isDone = true;
        }
        elec.setTurretMotor(speed);
        return isDone;
    }

    boolean turnFieldTurretOptimal(double goalAngle) {
        // This action will turn the turret toward a specific angle
        // based on the field point of view
        // Don't find the closest angle - find one that is mostly centered on our zero
        // positon
        double currentAngle = elec.getTurretFieldCenteredOnGoal(goalAngle);
        double difference = goalAngle - currentAngle;
        double actualGoal = elec.getTurretRotation() + difference;

        // if the goalAngle is past the soft limit, this moves the turret the opposite
        // way
        if (actualGoal > 250) {
            difference = difference - 360;
        }
        if (actualGoal < -250) {
            difference = difference + 360;
        }
        double speed = difference * DVRATIO;
        boolean isDone = false;

        if (speed > 0) {
            speed = Math.max(speed, LOWTURRETSPEED);
            speed = Math.min(speed, HIGHTURRETSPEED);
        }
        if (speed < 0) {
            speed = Math.min(speed, -LOWTURRETSPEED);
            speed = Math.max(speed, -HIGHTURRETSPEED);
        }
        if (Math.abs(difference) <= MINTURRETGOAL) {
            speed = 0;
            isDone = true;
        }
        elec.setTurretMotor(speed);
        return isDone;
    }

    boolean turnShoulder(double goalAngle) {
        // This action will turn the shoulder to a specific angle
        double currentAngle = elec.getShoulderRotation();
        double difference = goalAngle - currentAngle;
        boolean isDone = false;
        double ratio;
        double lowSpeed;
        double highSpeed;
        double minGoal;

        if (goalAngle > 175 && goalAngle < 185) {
            minGoal = 3.0;
        } else {
            minGoal = MINJ1GOAL;
        }

        if ((currentAngle > 180 && difference < 0)) {
            ratio = J1UNFOLDRATIO;
            lowSpeed = LOWJ1UNFOLDSPEED;
            highSpeed = HIGHJ1UNFOLDSPEED;
        } else if ((currentAngle <= 180 && difference > 0)) {
            ratio = J1RATIOUP;
            lowSpeed = LOWJ1SPEEDUP;
            highSpeed = HIGHJ1SPEEDUP;
            // Need more power to raise shoulder at these angles
            if (elec.getShoulderRotation() <= 135) {
                lowSpeed += 0.04;
            }
        } else {
            ratio = J1RATIODOWN;
            lowSpeed = LOWJ1SPEEDDOWN;
            highSpeed = HIGHJ1SPEEDDOWN;
        }
        double speed = difference * ratio;

        // Limit acceleration
        double oldShoulderVelocity = elec.getShoulderPower();
        if (speed > 0) {
            oldShoulderVelocity = Math.max(oldShoulderVelocity, 0);
            speed = Math.min(oldShoulderVelocity + (highSpeed - lowSpeed) * .05, speed);
        } else if (speed < 0) {
            oldShoulderVelocity = Math.min(oldShoulderVelocity, 0);
            speed = Math.max(oldShoulderVelocity - (highSpeed - lowSpeed) * .05, speed);
        }

        if (speed > 0) {
            speed = Math.max(speed, lowSpeed);
            speed = Math.min(speed, highSpeed);
        }
        if (speed < 0) {
            speed = Math.min(speed, -lowSpeed);
            speed = Math.max(speed, -highSpeed);
        }
        if (Math.abs(difference) <= minGoal) {
            speed = 0;
            isDone = true;
        }
        elec.setShoulderMotor(speed);
        return isDone;
    }

    boolean turnElbow(double goalAngle) {
        // This action will turn the elbow to a specific angle
        double currentAngle = elec.getElbowRotation();
        double difference = goalAngle - currentAngle;
        boolean isDone = false;
        double ratio;
        double lowSpeed;
        double highSpeed;

        if ((currentAngle >= 0 && currentAngle <= 180 && difference > 0) || (currentAngle > 180 && difference < 0)
                || (currentAngle <= 0 && difference < 0)) {
            ratio = J2RATIOUP;
            lowSpeed = LOWJ2SPEEDUP;
            highSpeed = HIGHJ2SPEEDUP;
        } else {
            ratio = J2RATIODOWN;
            lowSpeed = LOWJ2SPEEDDOWN;
            highSpeed = HIGHJ2SPEEDDOWN;
        }

        double speed = difference * ratio;

        // Limit acceleration
        double oldElbowVelocity = elec.getElbowPower();
        if (speed > 0) {
            oldElbowVelocity = Math.max(oldElbowVelocity, 0);
            speed = Math.min(oldElbowVelocity + (highSpeed - lowSpeed) * .05, speed);
        } else if (speed < 0) {
            oldElbowVelocity = Math.min(oldElbowVelocity, 0);
            speed = Math.max(oldElbowVelocity - (highSpeed - lowSpeed) * .05, speed);
        }

        if (speed > 0) {
            speed = Math.max(speed, lowSpeed);
            speed = Math.min(speed, highSpeed);
        }
        if (speed < 0) {
            speed = Math.min(speed, -lowSpeed);
            speed = Math.max(speed, -highSpeed);
        }
        if (Math.abs(difference) <= MINJ2GOAL) {
            speed = 0;
            isDone = true;
        }
        elec.setElbowMotor(speed);
        return isDone;
    }

    boolean turnWrist(double goalAngle) {
        // This action will turn the wrist to a specific angle
        double currentAngle = elec.getWristRotation();
        double difference = goalAngle - currentAngle;
        boolean isDone = false;
        double ratio;
        double lowSpeed;
        double highSpeed;

        if ((currentAngle <= 180 && difference > 0) || (currentAngle > 180 && difference < 0)) {
            ratio = WRATIOUP;
            lowSpeed = LOWWSPEEDUP;
            highSpeed = HIGHWSPEEDUP;
        } else {
            ratio = WRATIODOWN;
            lowSpeed = LOWWSPEEDDOWN;
            highSpeed = HIGHWSPEEDDOWN;
        }
        double speed = difference * ratio;
        if (speed > 0) {
            speed = Math.max(speed, lowSpeed);
            speed = Math.min(speed, highSpeed);
        }
        if (speed < 0) {
            speed = Math.min(speed, -lowSpeed);
            speed = Math.max(speed, -highSpeed);
        }
        if (Math.abs(difference) <= MINWGOAL) {
            speed = 0;
            isDone = true;
        }
        elec.setWristMotor(speed);
        return isDone;
    }

    boolean turnShoulderFast(double goalAngle) {
        // This action will turn the shoulder to a specific angle
        // Does not have acceleration limit
        double currentAngle = elec.getShoulderRotation();
        double difference = goalAngle - currentAngle;
        boolean isDone = false;
        double lowSpeed;
        double highSpeed;

        if ((currentAngle > 180 && difference < 0)) {
            lowSpeed = LOWJ1UNFOLDSPEED;
            highSpeed = HIGHJ1UNFOLDSPEED;
        } else if ((currentAngle <= 180 && difference > 0)) {
            lowSpeed = LOWJ1SPEEDUP;
            highSpeed = HIGHJ1SPEEDUP;
            // Need more power to raise shoulder at these angles
            if (elec.getShoulderRotation() <= 135) {
                lowSpeed += 0.04;
            }
        } else {
            lowSpeed = LOWJ1SPEEDDOWN;
            highSpeed = HIGHJ1SPEEDDOWN;
        }
        double speed;

        if (Math.abs(difference) <= 2.0) {
            speed = 0;
            isDone = true;
        } else if (difference > 0) {
            speed = (lowSpeed + 0.5 * (highSpeed - lowSpeed));
        } else {
            speed = -(lowSpeed + 0.5 * (highSpeed - lowSpeed));
        }

        elec.setShoulderMotor(speed);
        return isDone;
    }

    boolean turnElbowFast(double goalAngle) {
        // This action will turn the elbow to a specific angle
        // Does not have acceleration limit
        double currentAngle = elec.getElbowRotation();
        double difference = goalAngle - currentAngle;
        boolean isDone = false;
        double lowSpeed;
        double highSpeed;

        if ((currentAngle >= 0 && currentAngle <= 180 && difference > 0) || (currentAngle > 180 && difference < 0)
                || (currentAngle <= 0 && difference < 0)) {
            lowSpeed = LOWJ2SPEEDUP;
            highSpeed = HIGHJ2SPEEDUP;
        } else {
            lowSpeed = LOWJ2SPEEDDOWN;
            highSpeed = HIGHJ2SPEEDDOWN;
        }
        double speed;

        if (Math.abs(difference) <= 2.0) {
            speed = 0;
            isDone = true;
        } else if (difference > 0) {
            speed = (lowSpeed + 0.5 * (highSpeed - lowSpeed));
        } else {
            speed = -(lowSpeed + 0.5 * (highSpeed - lowSpeed));
        }

        elec.setElbowMotor(speed);
        return isDone;
    }

    boolean unFoldArm() {
        // The arm starts a match folded so the robot is less than 48" tall
        // This action unfolds the arm and puts it into driving position
        // Used at the start of a match during sandstorm
        switch (unFoldStage) {
        case stage1:
            if (elec.getShoulderRotation() < 270 || elec.getElbowRotation() < 120) {
                // No need to unfold, because the arm is already far enough unfolded -
                // just move the arm to the driving position
                return armToDrivePosition();
            }
            turnShoulder(drivingPositionAngle[0]);
            if (elec.getShoulderRotation() <= 340) {
                unFoldStage = UnFoldStage.stage2;
            }
            break;

        case stage2:
            turnElbow((elec.getShoulderRotation() - 160));
            if (turnShoulder(drivingPositionAngle[0])) {
                unFoldStage = UnFoldStage.stage3;
            }
            break;

        case stage3:
            if (turnShoulder(drivingPositionAngle[0]) & turnElbow(drivingPositionAngle[1])
                    & turnWrist(drivingPositionAngle[2])) {
                elec.setShoulderMotor(0);
                elec.setElbowMotor(0);
                elec.setWristMotor(0);
                return true;
            }
        }
        return false;
    }

    boolean grabHatch() {
        // This action will grab a hatch from the loading station
        switch (actionState) {

        case BEGIN: {
            elec.driveStraight(0);
            this.armToDrivePosition();
            this.turnWrist(drivingPositionWrist);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            double turretAngle = elec.getTurretFieldCenteredOnGoal(180);
            if (turretAngle <= 190 && turretAngle >= 170) {
                actionState = ActionState.TURRETTOTARGET;
                timer.reset();
                timer.start();
                rLog.print("Grab Hatch: Turret already close. Finding Target");
            } else {
                actionState = ActionState.TURNTURRET;
                rLog.print("Grab Hatch: Turn turret to 180");
            }
            setDashboardReason("");
            break;
        }

        case TURNTURRET: {
            elec.driveStraight(0); // Stop robot
            this.armToDrivePosition();
            this.turnWrist(drivingPositionWrist);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            double turretAngle = elec.getTurretFieldCenteredOnGoal(180);
            double turretDifference = Math.abs(turretAngle - 180);
            if (this.turnFieldTurret(180) || (turretDifference <= FASTVISIONANGLE && vision.foundTargets())) {
                actionState = ActionState.TURRETTOTARGET;
                timer.reset();
                timer.start();
                rLog.print("Grab Hatch: Turret close. Finding Target");
            }
            break;
        }

        case TURRETTOTARGET:
            elec.driveStraight(0); // Stop robot
            this.turnWrist(drivingPositionWrist);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            this.armToDrivePosition();
            elec.setTurretMotor(0);
            if (elec.getTurretVelocity() == 0 && (startFastTurretToTarget() || timer.get() > 0.3)) {
                rLog.print("Grab Hatch: Turret is still. Getting distance");
                actionState = ActionState.GETDISTANCE;
            }
            break;

        case GETDISTANCE:
            elec.driveStraight(0);
            elec.setTurretMotor(0);
            this.turnWrist(drivingPositionWrist);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            if (getFastTurretToTarget(180)) {
                double turretAngle = elec.getTurretFieldCenteredOnGoal(180);
                distance = vision.getDistance() * Math.cos(Math.toRadians(180 - turretAngle));
                double loadAngle = 180 - visionTurretTarget;
                rLog.print("Grab Hatch: Distance to Target: " + distance + " Angle = " + loadAngle);
                if (!vision.foundTargets()) {
                    assignGoalPosition(9, GRABHATCHHEIGHT);
                    actionState = ActionState.MANUALAIM;
                    rLog.print("Grab Hatch: Manual Aim");
                    setDashboardReason("Vision Failed. Use Manual Aim");
                } else if (distance >= 45) {
                    rLog.print("Grab Hatch: Too Far Away");
                    setDashboardReason("Too Far Away");
                    return true;
                } else if (distance - GRABBERLENGTHCLOSED < 8) {
                    rLog.print("Grab Hatch: Too Close");
                    setDashboardReason("Too Close");
                    return true;
                } else if (loadAngle >= 45) {
                    rLog.print("Grab Hatch: Too far to the left");
                    return true;
                } else if (loadAngle <= -45) {
                    rLog.print("Grab Hatch: Too far to the right");
                    return true;
                } else {
                    assignGoalPosition(distance - GRABBERLENGTHCLOSED - 3, GRABHATCHHEIGHT);
                    actionState = ActionState.MOVEARM1;
                    rLog.print("Grab Hatch: Moving to grabbing position 1");
                }
            }
            break;

        case MOVEARM1:
            elec.driveStraight(0);
            elec.setTurretMotor(0);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            this.turnWrist(90);
            boolean turretDone = turnFieldTurret(visionTurretTarget);
            if (turretDone && this.armToGoalPosition() && Math.abs(this.getCurrentX() - goalX) < 2.0
                    && Math.abs(this.getCurrentY() - goalY) < 2.0) {
                rLog.print("Grab Hatch: In grabbing position 1");
                assignGoalPosition(distance - GRABBERLENGTHCLOSED + 3.5, goalY);
                actionState = ActionState.MOVEARM2;
                timer.reset();
                timer.start();
            }
            if (turretDone && dS.getManualAimPressed()) {
                assignGoalPosition(distance - GRABBERLENGTHCLOSED - 3, GRABHATCHHEIGHT);
                actionState = ActionState.MANUALAIM;
            }
            break;

        case MOVEARM2:
            boolean isDone = false;
            elec.driveStraight(0);
            elec.setTurretMotor(0);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            this.turnWrist(90);
            double time;
            if (distance > 30) {
                time = 1.5;
            } else {
                time = 0.6;
            }
            if (this.armToGoalPosition() & timer.get() > time) {
                rLog.print("Grab Hatch: In grabbing position 2");
                isDone = true;
            } else if (timer.get() > 3.0) {
                rLog.print("Grab Hatch: Time's up, grab it");
                isDone = true;
            } else if (dS.getManualControlFinished()) {
                rLog.print("Grab Hatch: finish button presed / time: " + timer.get());
                isDone = true;
            }
            if (isDone) {
                actionState = ActionState.GRABHATCH;
                timer.reset();
                timer.start();
            } else if (dS.getManualAimPressed()) {
                assignGoalPosition(this.getCurrentX(), GRABHATCHHEIGHT);
                actionState = ActionState.MANUALAIM;
            }
            break;

        case GRABHATCH:
            elec.driveStraight(0);
            elec.setTurretMotor(0);
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            if ((this.turnWrist(105) && timer.get() > 0.5) || timer.get() > 0.8) {
                rLog.print("Grab Hatch: Wrist moved. Ready to remove arm");
                actionState = ActionState.WITHDRAWARM;
                assignGoalPosition(distance - GRABBERLENGTHCLOSED - 5.5, GRABHATCHHEIGHT + 4);
                rLog.print("Goal:" + goalShoulder + "," + goalElbow);
                timer.reset();
                timer.start();
            }
            break;

        case WITHDRAWARM:
            elec.driveStraight(0);
            elec.setTurretMotor(0);
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            this.turnWrist(105);
            if (this.turnShoulder(goalShoulder) & this.turnElbow(goalElbow)) {
                actionState = ActionState.DRIVINGPOSITION;
                rLog.print("Grab Hatch: Arm Withdrawn");
            }
            break;

        case DRIVINGPOSITION:
            this.allowDriving();
            if (this.armToDrivePosition() || dS.drivePressed()) {
                rLog.print("Grab Hatch: Arm back to driving position");
                actionState = ActionState.BACKUP;
            }
            break;

        case BACKUP:
            if (armToDrivePosition() & turnInPlaceToAngle(0, true)) {
                rLog.print("Grab hatch: Done turning robot. Ready to drive to tower.");
                return true;
            } else if (dS.drivePressed()) {
                rLog.print("Grab Hatch: Abort driving. Drive button pressed.");
                return true;
            }
            break;

        case MANUALAIM:
            this.allowDriving();
            manualControl(false); // Use slow speed for grabbig hathces
            this.turnWrist(90);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            if (dS.getManualControlFinished()) {
                distance = getCurrentX() + GRABBERLENGTHCLOSED - 3;
                rLog.print("Grab Hatch: Button: Finish Pressed.");
                actionState = ActionState.GRABHATCH;
                timer.reset();
                timer.start();
            }
            setDashboardReason("Manual Aim");
            break;

        default:
            rLog.print("Grab Hatch: Unknown state: " + actionState);
            break;
        }
        return false;
    }

    boolean grabCargo() {
        // This action will grab a cargo from the loading station
        switch (actionState) {

        case BEGIN: {
            elec.driveStraight(0); // Stop robot
            this.armToDrivePosition();
            double turretAngle = elec.getTurretFieldCenteredOnGoal(180);
            if (turretAngle <= 190 && turretAngle >= 170) {
                actionState = ActionState.TURRETTOTARGET;
                timer.reset();
                timer.start();
                rLog.print("Grab cargo: Turret already close. Finding Target");
            } else {
                actionState = ActionState.TURNTURRET;
                rLog.print("Grab cargo: Turn turret to 180");
            }
            setDashboardReason("");
            break;
        }

        case TURNTURRET: {
            elec.driveStraight(0); // Stop robot
            this.armToDrivePosition();
            double turretAngle = elec.getTurretFieldCenteredOnGoal(180);
            double turretDifference = Math.abs(turretAngle - 180);
            if (this.turnFieldTurret(180) || (turretDifference <= FASTVISIONANGLE && vision.foundTargets())) {
                actionState = ActionState.TURRETTOTARGET;
                timer.reset();
                timer.start();
                rLog.print("Grab cargo: Turret close. Finding Target");
            }
            break;
        }

        case TURRETTOTARGET:
            elec.driveStraight(0);
            elec.setGrabberMotor(GRABBERFULLYOPEN);
            elec.setTurretMotor(0);
            if (elec.getTurretVelocity() == 0 && (startFastTurretToTarget() || timer.get() > 0.3)) {
                rLog.print("Grab cargo: Turret is still. Getting Distance");
                actionState = ActionState.GETDISTANCE;
            }
            break;

        case GETDISTANCE:
            elec.driveStraight(0);
            elec.setTurretMotor(0);
            if (getFastTurretToTarget(180)) {
                double turretAngle = elec.getTurretFieldCenteredOnGoal(180);
                distance = vision.getDistance() * Math.cos(Math.toRadians(180 - turretAngle));
                double loadAngle = 180 - visionTurretTarget;
                rLog.print("Grab cargo: Distance to Target: " + distance + " Angle = " + loadAngle);
                if (!vision.foundTargets()) {
                    assignGoalPosition(9, GRABCARGOHEIGHT);
                    actionState = ActionState.MANUALAIM;
                    rLog.print("Grab cargo: Manual Aim");
                } else if (distance >= 45) {
                    rLog.print("Grab cargo: Too Far Away");
                    setDashboardReason("Too Far Away");
                    return true;
                } else if (distance - GRABBERLENGTHOPEN < 4) {
                    rLog.print("Grab cargo: Too Close");
                    setDashboardReason("Too Close");
                    return true;
                } else if (loadAngle >= 30) {
                    rLog.print("Grab cargo: Too far to the left");
                    return true;
                } else if (loadAngle <= -30) {
                    rLog.print("Grab cargo: Too far to the right");
                    return true;
                } else {
                    assignGoalPosition(distance - GRABBERLENGTHOPEN - 1, GRABCARGOHEIGHT);
                    actionState = ActionState.MOVEARM1;
                    rLog.print("Grab cargo: Moving to grabbing position 1");
                }
            }
            break;

        case MOVEARM1:
            elec.setGrabberMotor(GRABBERFULLYOPEN);
            boolean turretDone = turnFieldTurret(visionTurretTarget);
            if (this.armToGoalPosition() & this.turnWrist(GRABCARGOWRIST)) {
                rLog.print("Grab cargo: In grabbing position");
                actionState = ActionState.WAITFORBUTTON;
            }
            if (turretDone && dS.getManualAimPressed()) {
                actionState = ActionState.MANUALAIM;
            }
            break;

        case WAITFORBUTTON:
            manualControl(false); // Don't allow fast speed for grabbing cargo
            elec.driveStraight(0);
            if (dS.getManualControlFinished() || dS.getGrabBallPressed()) {
                rLog.print("Grab cargo: Button pressed");
                actionState = ActionState.CLOSEGRABBER;
                timer.reset();
                timer.start();
            }
            break;

        case CLOSEGRABBER:
            allowDriving();
            elec.setGrabberMotor(GRABBERHOLDINGCARGO);
            if (timer.get() > 1.5) {
                actionState = ActionState.BACKUP;
                rLog.print("Grab cargo: Grabber closed");
            }
            break;

        case BACKUP:
            armToDrivePosition();
            if (armToDrivePosition() & turnInPlaceToAngle(0, true)) {
                rLog.print("Grab cargo: Done turning robot. Ready to drive to tower.");
                return true;
            } else if (dS.drivePressed()) {
                rLog.print("Grab cargo: Abort finish. Drive button pressed.");
                return true;
            }
            break;

        case MANUALAIM:
            this.allowDriving();
            manualControl(false); // Don't allow fast speed for grabbing cargo
            this.turnWrist(GRABCARGOWRIST);
            elec.setGrabberMotor(GRABBERFULLYOPEN);
            if (dS.getManualControlFinished()) {
                rLog.print("Grab cargo: Finish Button pressed");
                actionState = ActionState.CLOSEGRABBER;
                timer.reset();
                timer.start();
            }
            setDashboardReason("Manual Aim");
            break;

        default:
            rLog.print("Grab cargo: Unknown state: " + actionState);
            break;
        }
        return false;
    }

    void allowDriving() {
        // Used for actions that allow driving the robot
        double magnitude;
        if (dS.drivePressed()) {
            if (dS.getSlowDrivePressed()) {
                magnitude = (dS.getMagnitude() - 0.05) / 3.0;
            } else {
                magnitude = (dS.getMagnitude() - 0.05);
            }
            if (dS.getTurnInPlacePressed()) {
                this.turnInPlaceToAngle(dS.getAngle(), true);
            } else if (dS.getMagnitude() > 0.05) {
                this.driveToAngle(dS.getAngle(), magnitude * 0.7);
            } else {
                elec.driveStraight(0); // Stop robot
            }
        } else {
            elec.driveStraight(0); // Stop robot
        }
    }

    void manualControl(boolean bFastInOut) {
        // Used for actions that allow the use of the manual aim buttons
        boolean left = dS.getManualControlLeft();
        boolean right = dS.getManualControlRight();
        boolean in = dS.getManualControlIn();
        boolean out = dS.getManualControlOut();
        double x = goalX;
        double y = goalY;
        double speed = -0.002 * x + 0.106;
        double xMin;
        double xMax;

        if (goalY >= 18) {
            xMin = 3;
        } else {
            xMin = 8;
        }

        if (goalY > 79) {
            xMax = 20;
        } else {
            xMax = 44 - GRABBERLENGTHCLOSED;
        }

        if (left) {
            elec.setTurretMotor(-speed);
        } else if (right) {
            elec.setTurretMotor(speed);
        } else {
            elec.setTurretMotor(0);
        }

        double inOutSpeed;
        if (bFastInOut) {
            inOutSpeed = 0.38;
        } else {
            inOutSpeed = 0.18;
        }

        if (in) {
            x = Math.max(x - inOutSpeed, xMin);
        } else if (out) {
            x = Math.min(x + inOutSpeed, xMax);
        } else {

        }

        assignGoalPosition(x, y);

        turnShoulder(goalShoulder);
        turnElbow(goalElbow);
    }

    boolean placeHatchRockeClose(AutoMode.Height height, AutoMode.Side side) {
        // This action will place a hatch into the rocket on the close side
        switch (actionState) {

        case BEGIN:
            rLog.print("Place Hatch Rocket Close: BEGIN");
            actionState = ActionState.PLACEHATCH;
            break;

        case PLACEHATCH:
            double angle = 0;
            if (side == AutoMode.Side.LEFT) {
                angle = -30;
            }
            if (side == AutoMode.Side.RIGHT) {
                angle = 30;
            }
            if (placeHatch(height, angle, true)) {
                if (!hatchWasPlaced) {
                    rLog.print("Place Hatch Rocket Close: hatch was not placed.");
                    return true;
                } else {
                    rLog.print("Place Hatch Rocket Close: Hatch was placed. Turning to loading station.");
                    actionState = ActionState.BACKUP;
                }
            }
            break;

        case BACKUP:
            armToDrivePosition();
            if (turnInPlaceToAngle(180, true)) {
                rLog.print("Place Hatch Rocket Close: Done turning robot. Ready to drive to loading station.");
                elec.resetDrive();
                return true;
            } else if (dS.drivePressed()) {
                rLog.print("Place Hatch Rocket Close: Abort driving. Drive button pressed.");
                return true;
            }

            break;

        default:
            rLog.print("Place Hatch Rocket Close: Unknown state: " + actionState);
            break;
        }
        return false;
    }

    boolean placeHatchRocketFar(AutoMode.Height height, AutoMode.Side side) {
        // This action will place a hatch into the rocket on the far side
        switch (actionState) {

        case BEGIN:
            rLog.print("Place Hatch Rocket Far: BEGIN");
            actionState = ActionState.PLACEHATCH;
            break;

        case PLACEHATCH:
            double angle = 0;
            if (side == AutoMode.Side.LEFT) {
                angle = -150;
            }
            if (side == AutoMode.Side.RIGHT) {
                angle = 150;
            }
            if (placeHatch(height, angle, true)) {
                if (!hatchWasPlaced) {
                    rLog.print("Place Hatch Rocket Far: hatch was not placed.");
                    return true;
                } else {
                    rLog.print("Place Hatch Rocket Far: hatch placed. Do not attempt to drive.");
                    actionState = ActionState.BACKUP;
                }
            }
            break;

        case BACKUP:
            if (armToDrivePosition()) {
                rLog.print("Place Hatch Rocket Far: Arm is now in driving position.");
                return true;
            } else if (dS.drivePressed()) {
                rLog.print("Place Hatch Rocket Far: Abort driving. Drive button pressed.");
                return true;
            }
            break;

        default:
            rLog.print("Place Hatch Rocket Far: Unknown state: " + actionState);
            break;
        }
        return false;
    }

    boolean placeHatch(AutoMode.Height height, double targetAngle, boolean waitForDrivingPosition) {
        // This action will place a hatch at the robot's current position
        switch (placeHatchState) {

        case BEGIN:
            setDashboardReason("");
            hatchWasPlaced = false;
            elec.driveStraight(0);
            double currentY = getCurrentY();
            assignGoalToCurrentPosition();
            if (currentY >= 34.0) {
                rLog.print("Place Hatch: Arm already high enough. X=" + round2(getCurrentX()) + " Y="
                        + round2(getCurrentY()));
                targetX = getCurrentX();
                targetY = currentY;
            } else {
                rLog.print("Place Hatch: Arm too low, moving to 3,34. X=" + round2(getCurrentX()) + " Y="
                        + round2(getCurrentY()));
                targetX = 3;
                targetY = 34.6;
            }
            targetWrist = drivingPositionWrist;
            targetSpeed = 0.5;
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            double turretAngle = elec.getTurretFieldCenteredOnGoal(targetAngle);
            if (turretAngle <= targetAngle + FASTVISIONANGLE && turretAngle >= targetAngle - FASTVISIONANGLE) {
                placeHatchState = PlaceHatchState.TURRETTOTARGET;
                elec.setTurretMotor(0);
                timer.reset();
                timer.start();
                rLog.print("Place Hatch: Turret already close. Finding Target");
            } else {
                placeHatchState = PlaceHatchState.TURNTURRET;
                rLog.print("Place Hatch: Turn turret to " + targetAngle);
            }
            break;

        case TURNTURRET:
            elec.driveStraight(0);
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            turretAngle = elec.getTurretFieldCenteredOnGoal(targetAngle);
            double turretDifference = Math.abs(turretAngle - targetAngle);
            if (this.turnFieldTurret(targetAngle) || (turretDifference <= FASTVISIONANGLE && vision.foundTargets())) {
                placeHatchState = PlaceHatchState.TURRETTOTARGET;
                elec.setTurretMotor(0);
                timer.reset();
                timer.start();
                rLog.print("Place Hatch: Turret close=" + round2(turretAngle) + ". Finding Target");
            }
            break;

        case TURRETTOTARGET:
            elec.driveStraight(0);
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            elec.setTurretMotor(0);
            // Wait for turret and robot to stop moving, and for arm to be out of the way
            if (elec.getTurretVelocity() == 0 && elec.getRightVelocity() == 0 && elec.getLeftVelocity() == 0
                    && this.getCurrentY() > 33.5) {
                rLog.print("Place Hatch: Turret and robot are still. Getting distance");
                placeHatchState = PlaceHatchState.GETDISTANCE;
                startFastTurretToTarget();
            }
            break;

        case GETDISTANCE:
            elec.driveStraight(0);
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            elec.setTurretMotor(0);
            if (getFastTurretToTarget(targetAngle)) {
                if (foundVisionTurretAngle) {
                    double loadAngle = targetAngle - visionTurretTarget;
                    rLog.print("Place Hatch: Distance to Target: " + distance + " Angle = " + visionTurretTarget
                            + " targetAngle=" + targetAngle);
                    if (distance >= 45) {
                        rLog.print("Place Hatch: Too Far Away");
                        setDashboardReason("Too Far Away");
                        return true;
                    } else if (distance - GRABBERLENGTHWITHHATCH < 8) {
                        rLog.print("Place Hatch: Too Close");
                        setDashboardReason("Too Close");
                        return true;
                    } else if (loadAngle >= 45) {
                        rLog.print("Place Hatch: Too far to the left");
                        return true;
                    } else if (loadAngle <= -45) {
                        rLog.print("Place Hatch: Too far to the right");
                        return true;
                    }
                    placeHatchState = PlaceHatchState.MOVEARMCLOSE;
                } else {
                    assignGoalToCurrentPosition();
                    placeHatchState = PlaceHatchState.PREMANUALAIM;
                    rLog.print("Place Hatch: Manual Aim");
                }
            }
            break;

        case MOVEARMCLOSE:
            elec.driveStraight(0);
            this.turnWrist(90);
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            if (height == Height.BOTTOM) {
                targetX = distance - GRABBERLENGTHWITHHATCH - 6;
                targetY = PUTHATCHLOWHEIGHT;
                targetSpeed = 0.3;
            } else if (height == Height.MIDDLE) {
                targetX = distance - GRABBERLENGTHWITHHATCH - 7;
                targetY = PUTHATCHMIDHEIGHT;
                targetSpeed = 0.52;
            } else {
                targetX = distance - GRABBERLENGTHWITHHATCH - 7;
                targetY = PUTHATCHHIGHHEIGHT;
                targetSpeed = 0.52;
            }
            this.moveToTargetPosition();
            boolean turretDone = turnFieldTurret(visionTurretTarget);
            if (turretDone && Math.abs(this.getCurrentX() - targetX) < 4.0
                    && Math.abs(this.getCurrentY() - targetY) < 2.5) {
                rLog.print("Place Hatch: Arm is close. Moving in");
                placeHatchState = PlaceHatchState.MOVEARMIN;
                timer.reset();
                timer.start();
            }
            if (turretDone && dS.getManualAimPressed()) {
                placeHatchState = PlaceHatchState.MANUALAIM;
            }
            break;

        case MOVEARMIN:
            boolean isDone = false;
            elec.driveStraight(0);
            this.turnWrist(90);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            this.turnFieldTurret(visionTurretTarget);
            targetX = distance - GRABBERLENGTHWITHHATCH + 7;
            this.moveToGoalPosition(targetX, targetY, 0.95);
            if (timer.get() > 2.6) {
                rLog.print("Place Hatch: Time's up, pulling out");
                isDone = true;
            } else if (elec.getShoulderVelocity() == 0 && elec.getElbowVelocity() == 0 && timer.get() > 1.6) {
                rLog.print("Place Hatch: Arm Stopped, pulling out, time=" + timer.get());
                isDone = true;
            } else if (dS.getManualControlFinished()) {
                rLog.print("Place Hatch: Finish button pressed, pulling out, time=" + timer.get());
                isDone = true;
            }
            if (isDone) {
                placeHatchState = PlaceHatchState.MOVEARMOUT;
                placedX = arm.getXFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                placedY = arm.getYFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                rLog.print("Place Hatch: PlacedX=" + round2(placedX) + " PlacedY=" + round2(placedY));
                placedWrist = elec.getWristRotation();
                timer.reset();
                timer.start();
            } else if (dS.getManualAimPressed()) {
                this.assignGoalToCurrentPosition();
                placeHatchState = PlaceHatchState.MANUALAIM;
            }
            break;

        case MOVEARMOUT:
            this.allowDriving();
            this.turnWrist(placedWrist - 5);
            elec.setGrabberMotor(GRABBERFULLYCLOSED);
            this.moveToGoalPosition(placedX - 10, placedY - 0.5, 0.5); // Move out
            if (dS.drivePressed() && timer.get() > 0.3) {
                turnFieldTurretOptimal(180);
            } else {
                elec.setTurretMotor(0);
            }
            double currentX = this.getCurrentX();
            if (currentX < placedX - 7) {
                placeHatchState = PlaceHatchState.DRIVINGPOSITION;
                rLog.print("Place Hatch: DONE in time:" + timer.get());
                hatchWasPlaced = true;
                if (!waitForDrivingPosition) {
                    return true;
                }
            } else if (timer.get() >= 3) {
                rLog.print("Place Hatch: Time's up, unable to pull out: current=" + currentX + " placed=" + placedX
                        + " goalX=" + goalX);
                elec.setAllMotorsOff();
                return true;
            }
            break;

        case DRIVINGPOSITION:
            this.allowDriving();
            if (dS.drivePressed()) {
                turnFieldTurretOptimal(180);
            } else {
                elec.setTurretMotor(0);
            }
            if (this.armToDrivePosition(0.3)) {
                rLog.print("Place Hatch: Arm back to driving position");
                return true;
            }
            break;

        case PREMANUALAIM:
            this.allowDriving();
            this.turnWrist(drivingPositionWrist);
            if (height == Height.BOTTOM) {
                targetX = 6;
                targetY = PUTHATCHLOWHEIGHT;
                targetSpeed = 0.3;
            } else if (height == Height.MIDDLE) {
                targetX = 3;
                targetY = PUTHATCHMIDHEIGHT;
                targetSpeed = 0.4;
            } else {
                targetX = 1;
                targetY = PUTHATCHHIGHHEIGHT;
                targetSpeed = 0.4;
            }
            this.moveToTargetPosition();
            if (Math.abs(getCurrentY() - targetY) < 3) {
                placeHatchState = PlaceHatchState.MANUALAIM;
                setDashboardReason("Manual Aim");
            }
            break;

        case MANUALAIM:
            this.allowDriving();
            this.turnWrist(90);
            if (height == Height.BOTTOM) {
                goalY = PUTHATCHLOWHEIGHT;
            } else if (height == Height.MIDDLE) {
                goalY = PUTHATCHMIDHEIGHT;
            } else {
                goalY = PUTHATCHHIGHHEIGHT;
            }
            manualControl(true); // Allow fast speed for placing hatches
            if (dS.getManualControlFinished()) {
                placeHatchState = PlaceHatchState.MOVEARMOUT;
                placedX = arm.getXFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                placedY = arm.getYFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                placedWrist = elec.getWristRotation();
                timer.reset();
                timer.start();
                rLog.print("Place Hatch: Button: Finish Pressed.");
            }
            setDashboardReason("Manual Aim");
            break;

        default:
            rLog.print("Place Hatch: Unknown state: " + placeHatchState);
            break;
        }
        return false;

    }

    boolean placeCargo(AutoMode.Height height, double targetAngle) {
        // This action will place a cargo at the robot's current position
        switch (placeHatchState) {

        case BEGIN:
            cargoWasPlaced = false;
            elec.driveStraight(0);
            if (height == Height.BOTTOM) {
                targetX = 3;
                targetY = 36;
            } else if (height == Height.MIDDLE) {
                targetX = 5;
                targetY = 40;
            } else {
                targetX = 3;
                targetY = 48;
            }
            targetWrist = drivingPositionWrist;
            targetSpeed = 0.5;
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            double turretAngle = elec.getTurretFieldCenteredOnGoal(targetAngle);
            if (turretAngle <= targetAngle + 5 && turretAngle >= targetAngle - 5) {
                placeHatchState = PlaceHatchState.TURRETTOTARGET;
                elec.setTurretMotor(0);
                timer.reset();
                timer.start();
                rLog.print("Place cargo: Turret already close. Finding Target");
            } else {
                placeHatchState = PlaceHatchState.TURNTURRET;
                rLog.print("Place cargo: Turn turret to " + targetAngle);
            }
            setDashboardReason("");
            break;

        case TURNTURRET:
            elec.driveStraight(0);
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            turretAngle = elec.getTurretFieldCenteredOnGoal(targetAngle);
            double turretDifference = Math.abs(turretAngle - targetAngle);
            if (this.turnFieldTurret(targetAngle) || (turretDifference <= FASTVISIONANGLE && vision.foundTargets())) {
                placeHatchState = PlaceHatchState.TURRETTOTARGET;
                elec.setTurretMotor(0);
                timer.reset();
                timer.start();
                rLog.print("Place cargo: Turret close. Finding Target");
            }
            break;

        case TURRETTOTARGET:
            elec.driveStraight(0);
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            elec.setTurretMotor(0);
            // Wait for turret and robot to stop moving, and for arm to be out of the way
            if (elec.getTurretVelocity() == 0 && elec.getRightVelocity() == 0 && elec.getLeftVelocity() == 0
                    && this.getCurrentY() > 34) {
                rLog.print("Place cargo: Turret and robot are still. Getting distance");
                placeHatchState = PlaceHatchState.GETDISTANCE;
                startFastTurretToTarget();
            }
            break;

        case GETDISTANCE:
            elec.driveStraight(0);
            this.moveToTargetPosition();
            this.turnWrist(targetWrist);
            elec.setTurretMotor(0);
            if (getFastTurretToTarget(targetAngle)) {
                if (foundVisionTurretAngle) {
                    double loadAngle = targetAngle - visionTurretTarget;
                    rLog.print("Place cargo: Distance to Target: " + distance + " Angle = " + visionTurretTarget
                            + " targetAngle=" + targetAngle);
                    double minDistance;
                    if (height == Height.BOTTOM) {
                        minDistance = GRABBERLENGTHWITHBALL + 6;
                    } else {
                        minDistance = GRABBERLENGTHWITHBALL + 4;
                    }
                    double maxDistance;
                    if (height == Height.TOP) {
                        maxDistance = 15 + GRABBERLENGTHWITHBALL + 1; // Arm does not reach further
                        loadAngle += 5;
                    } else {
                        maxDistance = 45;
                    }
                    if (distance > maxDistance) {
                        rLog.print("Place cargo: Too Far Away");
                        setDashboardReason("Too Far Away");
                        return true;
                    } else if (distance < minDistance) {
                        rLog.print("Place cargo: Too Close");
                        setDashboardReason("Too Close");
                        return true;
                    } else if (loadAngle >= 30) {
                        rLog.print("Place cargo: Too far to the left");
                        return true;
                    } else if (loadAngle <= -30) {
                        rLog.print("Place cargo: Too far to the right");
                        return true;
                    }
                    placeHatchState = PlaceHatchState.MOVEARMCLOSE;
                } else {
                    if (height == Height.BOTTOM) {
                        goalY = PUTCARGOLOWHEIGHT;
                    } else if (height == Height.MIDDLE) {
                        goalY = PUTCARGOMIDHEIGHT;
                    } else {
                        goalY = PUTCARGOHIGHHEIGHT;
                    }
                    assignGoalPosition(3, goalY);
                    placeHatchState = PlaceHatchState.MANUALAIM;
                    rLog.print("Place cargo: Manual Aim");
                }
            }
            break;

        case MOVEARMCLOSE:
            elec.driveStraight(0);
            this.turnWrist(90);
            elec.setGrabberMotor(GRABBERHOLDINGCARGO);
            if (height == Height.BOTTOM) {
                targetX = distance - GRABBERLENGTHINCLUDINGCARGO - 3;
                targetY = PUTCARGOLOWHEIGHT;
            } else if (height == Height.MIDDLE) {
                targetX = distance - GRABBERLENGTHINCLUDINGCARGO - 6;
                targetY = PUTCARGOMIDHEIGHT;
            } else {
                targetX = distance - GRABBERLENGTHINCLUDINGCARGO - 6;
                targetY = PUTCARGOHIGHHEIGHT;
            }
            targetSpeed = 0.4;
            this.moveToTargetPosition();
            boolean turretDone = turnFieldTurret(visionTurretTarget);
            if (turretDone && Math.abs(this.getCurrentX() - targetX) < 4.0
                    && Math.abs(this.getCurrentY() - targetY) < 1.5) {
                rLog.print("Place cargo: Arm is close");
                placeHatchState = PlaceHatchState.MOVEARMIN;
                timer.reset();
                timer.start();
            } else if (turretDone && dS.getManualAimPressed()) {
                placeHatchState = PlaceHatchState.MANUALAIM;
            }
            break;

        case MOVEARMIN:
            elec.driveStraight(0);
            this.turnWrist(90);
            elec.setGrabberMotor(GRABBERHOLDINGCARGO);
            this.turnFieldTurret(visionTurretTarget);
            targetX = distance - GRABBERLENGTHWITHBALL - 1.5;
            this.moveToGoalPosition(targetX, targetY, 0.95); // Move in
            if (this.getCurrentX() > targetX - 3) {
                rLog.print("Place cargo: Arm is in place to drop ball X=" + this.getCurrentX() + " goal=" + targetX
                        + " time=" + timer.get());
                placeHatchState = PlaceHatchState.OPENGRABBER;
                timer.reset();
                timer.start();
            } else if (dS.getManualControlFinished()) {
                rLog.print("Place cargo: Finish button pressed X=" + this.getCurrentX() + " goal=" + targetX + " time="
                        + timer.get());
                placeHatchState = PlaceHatchState.OPENGRABBER;
                timer.reset();
                timer.start();
            } else if (timer.get() > 3.5) {
                rLog.print("Place cargo: Time's up, need manual aim X=" + this.getCurrentX() + " goal=" + targetX);
                this.assignGoalPosition(targetX - 4, goalY);
                this.assignGoalToCurrentPosition();

                placeHatchState = PlaceHatchState.MANUALAIM;
            } else if (dS.getManualAimPressed()) {
                this.assignGoalPosition(targetX - 4, goalY);
                placeHatchState = PlaceHatchState.MANUALAIM;
            }
            break;

        case OPENGRABBER:
            elec.setAllMotorsOff();
            elec.setGrabberMotor(GRABBERFULLYOPEN);
            if (timer.get() > 0.25 && dS.getManualControlOut()) {
                rLog.print("Place carge: back arrow button pressed/time:" + timer.get());
                placeHatchState = PlaceHatchState.MOVEARMOUT;
                placedX = arm.getXFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                placedY = arm.getYFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                timer.reset();
                timer.start();
            } else if (timer.get() > 1.5 && timer.get() < 2.2) { // Do not set time below 1.5
                turnWrist(drivingPositionWrist);
            } else if (timer.get() > 2.5) {
                turnWrist(90);
                rLog.print("Place cargo: Done dropping ball");
                placeHatchState = PlaceHatchState.MOVEARMOUT;
                placedX = arm.getXFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                placedY = arm.getYFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
                timer.reset();
                timer.start();
            }
            break;

        case MOVEARMOUT:
            this.allowDriving();
            if (dS.drivePressed() && timer.get() > 1) {
                turnFieldTurretOptimal(180);
            } else {
                elec.setTurretMotor(0);
            }
            this.turnWrist(90);
            elec.setGrabberMotor(GRABBERFULLYOPEN);
            if (height == Height.BOTTOM) {
                targetX = placedX - 5;
            } else {
                targetX = placedX - 3;
            }
            this.moveToGoalPosition(targetX, placedY, 0.44); // Move out at fast speed
            double currentX = this.getCurrentX();
            if (currentX < targetX + 1) {
                placeHatchState = PlaceHatchState.DRIVINGPOSITION;
                rLog.print("Place cargo: DONE in time:" + timer.get());
                cargoWasPlaced = true;
            } else if (timer.get() >= 3) {
                rLog.print("Place cargo: Time's up, unable to pull out: current=" + currentX + " placed=" + placedX
                        + " goalX=" + goalX);
                elec.setAllMotorsOff();
                return true;
            }
            break;

        case DRIVINGPOSITION:
            this.allowDriving();
            if (dS.drivePressed()) {
                turnFieldTurretOptimal(180);
            } else {
                elec.setTurretMotor(0);
            }
            if (this.armToDrivePosition(0.3)) {
                rLog.print("Place cargo: Arm back to driving position");
                return true;
            }
            break;

        case MANUALAIM:
            elec.driveStraight(0);
            this.turnWrist(90);
            manualControl(false); // Don;t allow fast speed for placing cargo
            if (dS.getManualControlFinished()) {
                placeHatchState = PlaceHatchState.OPENGRABBER;
                timer.reset();
                timer.start();
                rLog.print("Place cargo: Button: Finish Pressed.");
            }
            setDashboardReason("Manual Aim");
            break;

        default:
            rLog.print("Place cargo: Unknown state: " + placeHatchState);
            break;
        }
        return false;

    }

    boolean placeBallRocket(AutoMode.Height height, AutoMode.Side side) {
        // This action will place a cargo at one of the rocket positions
        double goal = 1;
        if (side == AutoMode.Side.LEFT) {
            goal = -1;
        }
        if (side == AutoMode.Side.RIGHT) {
            goal = 1;
        }
        switch (actionState) {

        case BEGIN:
            rLog.print("Place Cargo Rocket: BEGIN");
            actionState = ActionState.PLACEBALL;
            break;

        case PLACEBALL:
            if (placeCargo(height, 90 * goal)) {
                if (!cargoWasPlaced) {
                    rLog.print("Place Cargo Rocket: Cargo was not placed.");
                    return true;
                } else {
                    rLog.print("Place Cargo Rocket: Cargo placed. Driving to loading station.");
                    actionState = ActionState.BACKUP;
                }
            }
            break;

        case BACKUP:
            if (armToDrivePosition()) {
                rLog.print("Arm is now in driving position after place cargo rocket.");
                return true; // Need to turn in place and then drive to loading station
            }
            break;

        default:
            rLog.print("Place Cargo Rocket: Unknown state: " + placeHatchState);
            break;
        }
        return false;
    }

    boolean sandStormFarRocket(AutoMode.Height height, AutoMode.Side side) {
        // This sandstorm program will drive to and place a hatch on the far side of
        // one of the rockets
        double goal = 1;
        if (side == AutoMode.Side.LEFT) {
            goal = -1;
        }
        if (side == AutoMode.Side.RIGHT) {
            goal = 1;
        }
        switch (actionState) {

        case BEGIN:
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            turnWrist(180);
            rLog.print("Sand Storm: place hatch far rocket");
            actionState = ActionState.DRIVESTRAIGHT;
            elec.resetDrive();
            break;

        case DRIVESTRAIGHT:
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            // Raise wrist to prevent hatch throwing hatch off
            turnWrist(180);
            // Drive the arm motors tight to prevent arm damage
            this.armDriveTight();
            driveToAngle(0, 0.05);
            if (elec.getRobotDisplacement() > 60) {
                elec.driveStraight(0);
                rLog.print("Drove off the habitat");
                actionState = ActionState.UNFOLD;
            }
            break;

        case UNFOLD:
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            if (this.unFoldArm() & turnInPlaceToAngle(28 * goal, false)) {
                rLog.print("Done unfolding and turning in place");
                actionState = ActionState.WAIT;
                elec.resetDrive();
            }
            break;

        case WAIT:
            elec.setAllMotorsOff();
            if (timer.get() > 0) {
                rLog.print("Done waiting 0 seconds");
                actionState = ActionState.DRIVETOTARGET;
            }
            break;

        case DRIVETOTARGET:
            turnTurret(90 * goal);
            if (driveDistanceToAngle(152, 28 * goal)) {
                rLog.print("Done driving to left of tower");
                actionState = ActionState.TURNINPLACE;
                elec.resetDrive();
            }
            break;

        case TURNINPLACE:
            turnTurret(90 * goal);
            if (turnInPlaceToAngle(60 * goal, false)) {
                rLog.print("Done turning");
                actionState = ActionState.LINEUP;
                elec.resetDrive();
            }
            break;

        case LINEUP:
            elec.setTurretMotor(0);
            if (driveDistanceToAngle(20, 60 * goal)) {
                rLog.print("Stopping Motors");
                elec.driveStraight(0);
                if (this.turretToTarget()) {
                    rLog.print("Lined up to place hatch far");
                    actionState = ActionState.PLACEHATCH;
                    placeHatchState = PlaceHatchState.GETDISTANCE;
                }
            }
            break;

        case PLACEHATCH:
            if (placeHatch(height, 150 * goal, false)) {
                actionState = ActionState.BACKUP;
            }
            break;

        case BACKUP:
            driveToAngle(180, 0.3);
            armToDrivePosition();
            if (Math.abs(elec.getRobotDisplacement()) > 40) {
                return true;
            }
            break;

        default:
            rLog.print("SandStorm Far Rocket: Unknown state: " + actionState);
            break;
        }

        return false;
    }

    boolean sandStormNearRocket(AutoMode.Height height, AutoMode.Side side) {
        // This sandstorm program will drive to and place a hatch on the near side of
        // one of the rockets
        double goal = 1;
        if (side == AutoMode.Side.LEFT) {
            goal = -1;
        }
        if (side == AutoMode.Side.RIGHT) {
            goal = 1;
        }

        switch (actionState) {

        case BEGIN:
            rLog.print("Sand Storm: place hatch near rocket");
            elec.resetGyro(90 * goal); // Robot starts facing the side
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            turnWrist(180);
            actionState = ActionState.DRIVESTRAIGHT;
            elec.resetDrive();
            break;

        case DRIVESTRAIGHT:
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            // Raise wrist to prevent hatch throwing hatch off
            turnWrist(180);
            // Drive the arm motors tight to prevent arm damage
            this.armDriveTight();
            driveToAngle(90 * goal, 0.08); // Drive off the side of habitat
            if (elec.getRobotDisplacement() > 48) {
                elec.driveStraight(0);
                rLog.print("Sand Storm: Drove off the habitat");
                actionState = ActionState.UNFOLD;
            }
            break;

        case UNFOLD:
            double actualGoal = 14.5 * goal;
            elec.setGrabberMotor(GRABBERHOLDINGHATCH);
            turnWrist(drivingPositionWrist);
            this.unFoldArm();
            if (turnInPlaceToAngle(actualGoal, false)) {
                rLog.print("Sand Storm: Done turning in place");
                actionState = ActionState.DRIVETOTARGET;
                elec.resetDrive();
            }
            break;

        case DRIVETOTARGET:
            if (height != Height.BOTTOM) {
                this.assignGoalPosition(7, PUTHATCHMIDHEIGHT);
                this.armToGoalPosition();
                turnWrist(drivingPositionWrist);
            } else {
                this.armToDrivePosition();
            }
            turnFieldTurret(30 * goal);
            if (driveDistanceToAngle(115, 14.5 * goal)) {
                rLog.print("Sand Storm: Done driving to front of tower");
                actionState = ActionState.PLACEHATCH;
            }
            break;

        case PLACEHATCH:
            if (placeHatch(height, 30 * goal, false)) {
                if (!hatchWasPlaced) {
                    rLog.print("Sand Storm: Hatch was not placed. Done with sandstorm.");
                    return true;
                } else {
                    rLog.print("Sand Storm: Hatch was placed. Driving to loading station");
                    actionState = ActionState.TURNINPLACE;
                }
            }
            break;

        case TURNINPLACE:
            actualGoal = 177 * goal;
            turnFieldTurret(180 * goal);
            this.armToDrivePosition(0.2);
            turnInPlaceToAngle(actualGoal, true);
            if (Math.abs(elec.getGyroCenteredOnGoal(actualGoal) - actualGoal) < 5) {
                rLog.print("Sand Storm: Done turining toward loading station");
                actionState = ActionState.BACKUP;
                elec.resetDrive();
            }
            break;

        case BACKUP:
            turnFieldTurret(180 * goal);
            armToDrivePosition(0.2);
            if (driveDistanceToAngle(150, 177 * goal)) {
                rLog.print("Sand Storm: Done driving to loading station. Done with sandstorm.");
                return true;
            }
            break;

        default:
            rLog.print("Sand Storm Near Rocket: Unknown state: " + actionState);
            break;
        }

        return false;
    }

    double getCurrentX() {
        // Get the X locaiton of arm based on shoulder//elbow angles
        double currentX = arm.getXFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
        return currentX;
    }

    double getCurrentY() {
        // Get the Y locaiton of arm based on shoulder//elbow angles
        double currentY = arm.getYFromAngle(elec.getShoulderRotation(), elec.getElbowRotation());
        return currentY;
    }

    boolean moveToTargetPosition() {
        return moveToGoalPosition(targetX, targetY, targetSpeed);
    }

    boolean moveToGoalPosition(double x, double y, double speed) {
        double newX = goalX;
        double newY = goalY;

        if (Math.abs(x - goalX) < speed) {
            newX = x;
        } else if (x > goalX) {
            newX += speed;
        } else if (x < goalX) {
            newX -= speed;
        }
        if (Math.abs(y - goalY) < speed) {
            newY = y;
        } else if (y > goalY) {
            newY += speed;
        } else if (y < goalY) {
            newY -= speed;
        }
        assignGoalPosition(newX, newY);
        return armToGoalPosition();
    }

    boolean moveToGoalPositionFast(double x, double y, double speed) {
        double newX = goalX;
        double newY = goalY;

        if (Math.abs(x - goalX) < speed) {
            newX = x;
        } else if (x > goalX) {
            newX += speed;
        } else if (x < goalX) {
            newX -= speed;
        }
        if (Math.abs(y - goalY) < speed) {
            newY = y;
        } else if (y > goalY) {
            newY += speed;
        } else if (y < goalY) {
            newY -= speed;
        }
        assignGoalPosition(newX, newY);
        return armToGoalPositionFast();
    }

    void assignGoalToCurrentPosition() {
        assignGoalAngle(elec.getShoulderRotation(), elec.getElbowRotation());
    }

    void assignGoalAngle(double shoulder, double elbow) {
        goalShoulder = shoulder;
        goalElbow = elbow;
        goalX = arm.getXFromAngle(shoulder, elbow);
        goalY = arm.getYFromAngle(shoulder, elbow);
    }

    void assignGoalPosition(double x, double y) {
        goalX = x;
        goalY = y;
        goalShoulder = arm.getShoulderAngleFromXY(x, y);
        goalElbow = arm.getElbowAngleFromXY(x, y, goalShoulder);
    }

    void setTargetArm(double x, double y, double speed) {
        targetX = x;
        targetY = y;
        targetSpeed = speed;
    }

    void setUpWrist() {
        turnWrist(98);
        elec.setGrabberMotor(0.4);
    }

    void testDrivePerformance() {
        double values[] = { -0.4, -0.3, -0.25, -0.2, -0.18, -0.16, -0.14, -0.12, -0.1, -0.08, 0.08, 0.1, 0.12, 0.14,
                0.16, 0.18, .2, .25, .3, .4 };
        rLog.print("Test Performance; Left: " + ",Pow:" + Math.round(elec.getLeftPower() * 1000.0) / 1000.0 + ",Vel:"
                + elec.getLeftVelocity() + " Right: " + ",Pow:" + Math.round(elec.getRightPower() * 1000.0) / 1000.0
                + ",Vel:" + elec.getRightVelocity());
        for (int i = 0; i < 20; i++) {
            if (timer.get() < 2 * (i + 1)) {
                elec.setLeftMotorExact(values[i]);
                elec.setRightMotorExact(values[i]);
                return;
            }
        }
        elec.setLeftMotorExact(0.2);
        elec.setRightMotorExact(0.2);
    }
}
