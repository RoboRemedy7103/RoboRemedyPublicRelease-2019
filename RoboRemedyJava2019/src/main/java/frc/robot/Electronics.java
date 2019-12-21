/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import java.io.File;

/**
 * This class sets the motors, gyro, sensors, etc.
 */
public class Electronics {

    private TalonSRX leftMotor1 = new TalonSRX(1); // Gyro
    private TalonSRX leftMotor2 = new TalonSRX(2); // Encoder
    private TalonSRX rightMotor2 = new TalonSRX(3);
    private TalonSRX rightMotor1 = new TalonSRX(4); // Encoder
    private TalonSRX turretMotor = new TalonSRX(5);
    private TalonSRX shoulderMotor = new TalonSRX(6);
    private TalonSRX wristMotor = new TalonSRX(7);
    private TalonSRX elbowMotor1 = new TalonSRX(8);
    private VictorSPX elbowMotor2 = new VictorSPX(9);
    private Servo grabberMotor = new Servo(1);
    private PigeonIMU gyro;
    private AHRS gyro2;
    private double[] gyroValue = new double[3];
    private DigitalOutput lightLED = new DigitalOutput(0);
    Compressor compressor;
    DoubleSolenoid solenoid;
    RobotName robot;

    private static double TURRETRDRATIO = 0.00046263;
    private static final double SHOULDERRATIO = 0.087890625;
    private static final double ELBOWRATIO = 0.087890625;
    private static final double WRISTRATIO = 0.087890625;
    private static final double DRIVERAMP = 0.08;
    private static final double DRIVEADJRAMP = 0.1;
    private static final double TURRETRAMP = 3.0;
    private static final double SHOULDERRAMP = 0.5;
    private static final double ELBOWRAMP = 0.5;
    private static final double WRISTRAMP = 0.2;
    public static final double FORWARDTHRESHOLD = 450;
    public static final double REVERSETHRESHOLD = -450;
    private static final double FORWARDELBOW = 180;
    private static final double REVERSEELBOW = 20;
    private static final double FORWARDWRIST = 270;
    private static final double REVERSEWRIST = 90;
    private static final double SHOULDEROFFSET = 299.2;
    private static final double ELBOWOFFSET = 291.22;
    private static final double WRISTOFFSET = -46.2;
    private static final double DRIVERATIO = 0.0009151; // TestRobot = 0.0045261; // Duluth = 0.0009151;

    public static double MAXTURRETAMPS = 10;
    public static double MAXSHOULDERAMPS = 10;
    public static double MAXELBOWAMPS = 10;
    public static double MAXWRISTAMPS = 10;
    public static double MAXGRABBERAMPS = 10;
    double totalShoulderAmps = 0;
    double totalTurretAmps = 0;
    double totalElbowAmps = 0;
    double totalWristAmps = 0;

    // This is for the Live robot
    public static double[] leftMotorPower = { -0.9, -0.7, -0.5, -0.4, -0.3, -0.2, -0.19, 0.25, 0.2, 0.3, 0.4, 0.6, 0.8,
            1.0 };
    public static double[] rightMotorPower = { -0.9, -0.7, -0.5, -0.4, -0.3, -0.2, -0.17, 0.22, 0.2, 0.3, 0.4, 0.6, 0.8,
            1.0 };
    public static double[] leftVelocity = { -1.1091, -0.8162, -0.5131, -0.3616, -0.2020, -0.0485, 0, 0, 0.0384, 0.1778,
            0.3232, 0.4646, 0.7273, 1 };
    public static double[] rightVelocity = { -1.0808, -0.796, -0.5152, -0.3636, -0.2172, -0.0606, 0, 0, 0.0606, 0.1919,
            0.3535, 0.4848, 0.7677, 1.0606 };

    // This is for the Practice robot (Mr J)
    // public static double[] leftMotorPower = { -0.9, -0.7, -0.5, -0.4, -0.3, -0.2,
    // -0.15, 0.14, 0.2, 0.3, 0.4, 0.5, 0.7,
    // 0.9 };
    // public static double[] rightMotorPower = { -0.9, -0.7, -0.5, -0.4, -0.3,
    // -0.2, -0.15, 0.14, 0.2, 0.3, 0.4, 0.5, 0.7,
    // 0.9 };
    // public static double[] leftVelocity = { -1.1091, -0.8162, -0.5131, -0.3616,
    // -0.2020, -0.0485, 0, 0, 0.0384, 0.1778,
    // 0.3232, 0.4646, 0.7273, 1 };
    // public static double[] rightVelocity = { -1.1091, -0.8162, -0.5131, -0.3616,
    // -0.2020, -0.0485, 0, 0, 0.0384, 0.1778,
    // 0.3232, 0.4646, 0.7273, 1 };

    public enum RobotName {
        LIVEROBOT, MRJ, TESTROBOT
    }

    public static void main(String[] args) {
        System.out.print("Testing for speed calculations");
        for (int i = 0; i < 14; i++) {
            System.out.println("Speed(" + rightVelocity[i] + ") = " + calcRightAdjSpeed(rightVelocity[i]));
        }
        for (double s = -1.0; s < 1.0; s += 0.01) {
            System.out.println("Speed(" + s + ")=" + calcRightAdjSpeed(s));
        }
    }

    Electronics() {
        // We only have one claw, but there are three robot bases that can run this code
        // To use the same code for all three robots, check to see if one of these two
        // files exist:
        // If TestRobot.txt exists in the /home/lvuser direcory, then this is "Test
        // Rebot".
        // If MrJ.txt exists in the /home/lvuser directory, then this is the "Mr. J"
        // robot.
        // Otherwise this is the "Live Robot"
        File t = new File("/home/lvuser/TestRobot.txt");
        File j = new File("/home/lvuser/MrJ.txt");

        if (t.isFile()) {
            System.out.println("Robot = Test Robot");
            robot = RobotName.TESTROBOT;
        } else if (j.isFile()) {
            System.out.println("Robot = Mr. J.");
            robot = RobotName.MRJ;
        } else {
            System.out.println("Robot = Live Robot");
            robot = RobotName.LIVEROBOT;
        }
        if (robot == RobotName.LIVEROBOT) {
            compressor = new Compressor(0);
            solenoid = new DoubleSolenoid(0, 1);
        }
        if (robot == RobotName.LIVEROBOT) {
            leftMotor1.setInverted(true);
            leftMotor2.setInverted(true);
            rightMotor1.setInverted(false);
            rightMotor2.setInverted(false);
            leftMotor2.setSensorPhase(false);
            rightMotor2.setSensorPhase(false);
        } else {
            leftMotor1.setInverted(false);
            rightMotor1.setInverted(true);
            leftMotor2.setInverted(false);
            rightMotor2.setInverted(true);
            leftMotor2.setSensorPhase(true);
            rightMotor2.setSensorPhase(true);
        }
        if (robot == RobotName.LIVEROBOT) {
            gyro = new PigeonIMU(leftMotor1);
            gyro2 = new AHRS(SPI.Port.kMXP);
        } else if (robot == RobotName.MRJ) {
            gyro = new PigeonIMU(rightMotor1);
        } else {
            gyro2 = new AHRS(SPI.Port.kMXP);
        }
        turretMotor.setInverted(true);
        shoulderMotor.setInverted(false);
        wristMotor.setInverted(true);
        elbowMotor1.setInverted(true);
        elbowMotor2.setInverted(true);
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Brake);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        elbowMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        turretMotor.setSensorPhase(false);
        shoulderMotor.setSensorPhase(false);
        wristMotor.setSensorPhase(true);
        elbowMotor1.setSensorPhase(true);
        leftMotor2.configOpenloopRamp(DRIVERAMP, 0);
        rightMotor2.configOpenloopRamp(DRIVERAMP, 0);
        turretMotor.configOpenloopRamp(TURRETRAMP, 0);
        shoulderMotor.configOpenloopRamp(SHOULDERRAMP, 0);
        wristMotor.configOpenloopRamp(ELBOWRAMP, 0);
        elbowMotor1.configOpenloopRamp(WRISTRAMP, 0);
        turretMotor.overrideLimitSwitchesEnable(false);
        turretMotor.configForwardSoftLimitEnable(true, 10);
        turretMotor.configForwardSoftLimitThreshold((int) (FORWARDTHRESHOLD / TURRETRDRATIO), 10);
        turretMotor.configReverseSoftLimitEnable(true, 10);
        turretMotor.configReverseSoftLimitThreshold((int) (REVERSETHRESHOLD / TURRETRDRATIO), 10);
        shoulderMotor.configForwardSoftLimitEnable(false, 10);
        shoulderMotor.configReverseSoftLimitEnable(false, 10);
        shoulderMotor.overrideLimitSwitchesEnable(true);
        wristMotor.configForwardSoftLimitEnable(false, 10);
        wristMotor.configForwardSoftLimitThreshold((int) (FORWARDWRIST / WRISTRATIO), 10);
        wristMotor.configReverseSoftLimitEnable(false, 10);
        wristMotor.configReverseSoftLimitThreshold((int) (REVERSEWRIST / WRISTRATIO), 10);
        elbowMotor1.configReverseSoftLimitEnable(false, 10);
        elbowMotor1.configReverseSoftLimitThreshold((int) (FORWARDELBOW / ELBOWRATIO), 10);
        elbowMotor1.configReverseSoftLimitEnable(false, 10);
        elbowMotor1.configReverseSoftLimitThreshold((int) (REVERSEELBOW / ELBOWRATIO), 10);
        leftMotor1.follow(leftMotor2);
        rightMotor1.follow(rightMotor2);
        elbowMotor2.follow(elbowMotor1);
    }

    void setLeftMotorExact(double leftMotorVelocity) {
        // left motor 1 is set to follow left motor 2
        leftMotor2.set(ControlMode.PercentOutput, leftMotorVelocity);
    }

    void setRightMotorExact(double rightMotorVelocity) {
        // right motor 1 is set to follow right motor 2
        rightMotor2.set(ControlMode.PercentOutput, rightMotorVelocity);
    }

    static double calcLeftAdjSpeed(double input) {
        double slope;
        double output = 0;
        if (input == 0) {
            output = 0;
        } else if (input >= leftVelocity[13]) {
            output = leftMotorPower[13];
        } else if (input <= leftVelocity[0]) {
            output = leftMotorPower[0];
        } else
            for (int i = 0; i < 13; i++) {
                if (input >= leftVelocity[i] && input <= leftVelocity[i + 1]) {
                    slope = (leftMotorPower[i + 1] - leftMotorPower[i]) / (leftVelocity[i + 1] - leftVelocity[i]);
                    output = slope * (input - leftVelocity[i]) + leftMotorPower[i];
                    break;
                }
            }
        return output;
    }

    void setLeftAdjSpeed(double input) {
        double newSpeed = calcLeftAdjSpeed(input);
        double oldSpeed = getLeftPower();
        newSpeed = Math.min(oldSpeed + DRIVEADJRAMP, newSpeed);
        newSpeed = Math.max(oldSpeed - DRIVEADJRAMP, newSpeed);
        leftMotor2.set(ControlMode.PercentOutput, newSpeed);
    }

    static double calcRightAdjSpeed(double input) {
        double slope;
        double output = 0;
        if (input == 0) {
            output = 0;
        } else if (input >= rightVelocity[13]) {
            output = rightMotorPower[13];
        } else if (input <= rightVelocity[0]) {
            output = rightMotorPower[0];
        } else
            for (int i = 0; i < 13; i++) {
                if (input >= rightVelocity[i] && input <= rightVelocity[i + 1]) {
                    slope = (rightMotorPower[i + 1] - rightMotorPower[i]) / (rightVelocity[i + 1] - rightVelocity[i]);
                    output = slope * (input - rightVelocity[i]) + rightMotorPower[i];
                    break;
                }
            }
        return output;
    }

    void setRightAdjSpeed(double input) {
        double newSpeed = calcRightAdjSpeed(input);
        double oldSpeed = getRightPower();
        newSpeed = Math.min(oldSpeed + DRIVEADJRAMP, newSpeed);
        newSpeed = Math.max(oldSpeed - DRIVEADJRAMP, newSpeed);
        rightMotor2.set(ControlMode.PercentOutput, newSpeed);
    }

    void setTurretMotor(double turretMotorVelocity) {
        turretMotor.set(ControlMode.PercentOutput, turretMotorVelocity);
    }

    void setShoulderMotor(double shoulderVelocity) {
        // Soft Limits
        if ((this.getShoulderRotation() >= 350 && shoulderVelocity >= 0)
                || (this.getShoulderRotation() <= 82 && shoulderVelocity <= 0)) {
            shoulderVelocity = 0;
        }

        // Gravity Assist
        if (this.getShoulderRotation() < 125 && shoulderVelocity >= 0 && shoulderVelocity < 0.055) {
            // Prevent shoulder from falling when we want it to not move
            shoulderVelocity = 0.055;
        } else if (this.getShoulderRotation() > 225 && this.getShoulderRotation() < 335 && shoulderVelocity <= 0
                && shoulderVelocity > -0.055) {
            shoulderVelocity = -0.055;
        }

        shoulderMotor.set(ControlMode.PercentOutput, shoulderVelocity);
    }

    void setElbowMotor(double armMotor2Velocity) {

        // Gravity Assist
        if (this.getElbowRotation() > 64 && this.getElbowRotation() < 124 && armMotor2Velocity >= 0
                && armMotor2Velocity < 0.035) {
            // Prevent elbow from falling when we want it to not move
            armMotor2Velocity = 0.035;
        }

        elbowMotor1.set(ControlMode.PercentOutput, armMotor2Velocity);
    }

    void setWristMotor(double wristMotorVelocity) {
        wristMotor.set(ControlMode.PercentOutput, wristMotorVelocity);
    }

    boolean setGrabberMotor(double grabberMotorValue) {
        double lastGrabber = getGrabberPosition();
        if (grabberMotorValue > lastGrabber) {
            grabberMotor.set(lastGrabber + 0.01);
        } else if (grabberMotorValue < lastGrabber) {
            grabberMotor.set(lastGrabber - 0.01);
        } else if (lastGrabber == grabberMotorValue) {
            return true;
        } else {
            grabberMotor.set(grabberMotorValue);
        }
        return false;
    }

    void setAllMotorsOff() {
        driveStraight(0);
        setTurretMotor(0);
        setShoulderMotor(0);
        setElbowMotor(0);
        setWristMotor(0);
    }

    void enableCompressor() {
        if (robot == RobotName.LIVEROBOT) {
            compressor.setClosedLoopControl(true);
        }
    }

    void disableCompressor() {
        if (robot == RobotName.LIVEROBOT) {
            compressor.setClosedLoopControl(false);
        }
    }

    double getLeftRotation() {
        return leftMotor2.getSelectedSensorPosition(0) * DRIVERATIO;
    }

    double getRightRotation() {
        return rightMotor2.getSelectedSensorPosition(0) * DRIVERATIO;
    }

    double getLeftVelocity() {
        return leftMotor2.getSelectedSensorVelocity(0) * DRIVERATIO;
    }

    double getRightVelocity() {
        return rightMotor2.getSelectedSensorVelocity(0) * DRIVERATIO;
    }

    double getTurretVelocity() {
        return turretMotor.getSelectedSensorVelocity(0);
    }

    double getShoulderVelocity() {
        return shoulderMotor.getSelectedSensorVelocity(0);
    }

    double getElbowVelocity() {
        return elbowMotor1.getSelectedSensorVelocity();
    }

    double getRobotDisplacement() {
        double displacement = (getLeftRotation() + getRightRotation()) / 2;
        return displacement;
    }

    double getLeftPower() {
        return leftMotor2.getMotorOutputPercent();
    }

    double getRightPower() {
        return rightMotor2.getMotorOutputPercent();
    }

    double getShoulderPower() {
        return shoulderMotor.getMotorOutputPercent();
    }

    double getElbowPower() {
        return elbowMotor1.getMotorOutputPercent();
    }

    double getWristPower() {
        return wristMotor.getMotorOutputPercent();
    }

    double getTurretPower() {
        return turretMotor.getMotorOutputPercent();
    }

    double getTurretRotation() {
        return turretMotor.getSelectedSensorPosition(0) * TURRETRDRATIO;
    }

    double getTurretFieldCenteredOnGoal(double goalAngle) {
        double value = getGyro() + getTurretRotation();

        while (value < (goalAngle - 180)) {
            value += 360;
        }
        while (value > (goalAngle + 180)) {
            value -= 360;
        }

        return value;
    }

    double getMaxTurret() {
        return FORWARDTHRESHOLD;
    }

    double getMinTurret() {
        return REVERSETHRESHOLD;
    }

    double getShoulderRotation() {
        double value = shoulderMotor.getSelectedSensorPosition(0) * SHOULDERRATIO + SHOULDEROFFSET;
        while (value > 360)
            value -= 360;
        while (value < 0)
            value += 360;
        return value;
    }

    double getElbowRotation() {
        double value = shoulderMotor.getSelectedSensorPosition(0) * SHOULDERRATIO
                + elbowMotor1.getSelectedSensorPosition(0) * ELBOWRATIO + ELBOWOFFSET;
        while (value > 270)
            value -= 360;
        while (value < -90)
            value += 360;
        return value;
    }

    double getWristRotation() {
        double value = shoulderMotor.getSelectedSensorPosition(0) * SHOULDERRATIO
                + elbowMotor1.getSelectedSensorPosition(0) * ELBOWRATIO
                + wristMotor.getSelectedSensorPosition(0) * WRISTRATIO + WRISTOFFSET;
        while (value > 300) {
            value -= 360;
        }
        while (value < -60) {
            value += 360;
        }
        return value;
    }

    double getGrabberPosition() {
        return grabberMotor.get();
    }

    double getLeftAmps() {
        return leftMotor1.getOutputCurrent() + leftMotor2.getOutputCurrent();
    }

    double getRightAmps() {
        return rightMotor1.getOutputCurrent() + rightMotor2.getOutputCurrent();
    }

    double getTurretAmps() {
        return turretMotor.getOutputCurrent();
    }

    double getShoulderAmps() {
        return shoulderMotor.getOutputCurrent();
    }

    double getElbowAmps() {
        return elbowMotor1.getOutputCurrent();
    }

    double getWristAmps() {
        return (2.0 * wristMotor.getOutputCurrent());
    }

    void driveStraight(double motorVelocity) {
        this.setLeftAdjSpeed(motorVelocity);
        this.setRightAdjSpeed(motorVelocity);
    }

    void resetGyro(double startAngle) {
        if (robot == RobotName.TESTROBOT) {
        } else {
            gyro.setYaw(-startAngle, 0);
            gyro2.reset();
        }
    }

    double getGyro() {
        if (robot == RobotName.TESTROBOT) {
            return 0;
        } else {
            gyro.getYawPitchRoll(gyroValue);
            // return -gyroValue[0]; // Use the Pigeon
            return gyro2.getAngle(); // Use the NavX
        }
    }

    double getGyroCenteredOnGoal(double goalAngle) {
        double gyroValue = getGyro();

        while (gyroValue < (goalAngle - 180)) {
            gyroValue += 360;
        }
        while (gyroValue > (goalAngle + 180)) {
            gyroValue -= 360;
        }

        return gyroValue;
    }

    void resetTurret() {
        turretMotor.setSelectedSensorPosition((int) (0 / TURRETRDRATIO), 0, 0);
    }

    void resetDrive() {
        leftMotor2.setSelectedSensorPosition(0, 0, 0);
        rightMotor2.setSelectedSensorPosition(0, 0, 0);
    }

    void setLED(boolean on) {
        // Green light for vision processing
        if (on) {
            lightLED.set(false);
        } else {
            lightLED.set(true);
        }
    }
}
