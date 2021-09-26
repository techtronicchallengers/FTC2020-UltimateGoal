package org.firstinspires.ftc.teamcode.assembly;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class RobotHardware
{
    //Chassis
    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    public DcMotor intaker = null;
    public DcMotor lift = null;
    public DcMotor shooter = null;
    public DcMotor arm = null;

    public Servo pusher = null;
    public CRServo guideWheel = null;
    public Servo shooterAngleServo = null;
    public Servo gripper = null;
    public Servo door = null;
    public Servo guard = null;


    //Webcam
    public WebcamName webcam = null;

    public BNO055IMU imu;

    //Distance Sensors
    Rev2mDistanceSensor frontRightSensor = null;
    Rev2mDistanceSensor backRightSensor = null;
    Rev2mDistanceSensor frontLeftSensor = null;
    Rev2mDistanceSensor backLeftSensor = null;

    //Distance Sensors
    public ModernRoboticsI2cRangeSensor rightFrontSensor = null;
    public ModernRoboticsI2cRangeSensor rightBackSensor = null;
    public ModernRoboticsI2cRangeSensor backSensor = null;

    Rev2mDistanceSensor backLaser = null;
    Rev2mDistanceSensor frontLaser = null;

    //Touch Sensors
    public TouchSensor bottomTouch = null;
    public TouchSensor topTouch = null;
    public TouchSensor armReturn = null;
    public TouchSensor grabTouch = null;

    //Adding the Hardware Map
    private HardwareMap hwMap  = null;

    public  RobotHardware(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        //Wheel motors
        frontLeftWheel = hwMap.get(DcMotor.class, "frontLeft");
        frontRightWheel = hwMap.get(DcMotor.class, "frontRight");
        backLeftWheel = hwMap.get(DcMotor.class, "backLeft");
        backRightWheel = hwMap.get(DcMotor.class, "backRight");

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        frontLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        frontRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        intaker = hwMap.get(DcMotor.class, "intaker");
        intaker.setDirection(DcMotor.Direction.FORWARD);
        intaker.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));

        lift = hwMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        shooter = hwMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));

        arm = hwMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pusher = hwMap.get(Servo.class, "pusher");
        guideWheel = hwMap.get(CRServo.class, "guideWheel");
        shooterAngleServo = hwMap.get(Servo.class, "shooterAngleServo");
        gripper = hwMap.get(Servo.class, "gripper");
        door = hwMap.get(Servo.class, "door");
        guard = hwMap.get(Servo.class, "guard");


        bottomTouch = hwMap.get(TouchSensor.class, "bottomTouch");
        topTouch = hwMap.get(TouchSensor.class, "topTouch");
        armReturn = hwMap.get(TouchSensor.class, "armReturn");
        grabTouch = hwMap.get(TouchSensor.class, "grabTouch");

        webcam = hwMap.get(WebcamName.class, "webcam");
        imu = hwMap.get(BNO055IMU.class, "imu");

        rightFrontSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rightFrontSensor");
        rightBackSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rightBackSensor");
        backSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "backSensor");

/*
        frontRightSensor = hwMap.get(Rev2mDistanceSensor.class, "frontRightSensor");
        backRightSensor = hwMap.get(Rev2mDistanceSensor.class, "backRightSensor");

        frontLeftSensor = hwMap.get(Rev2mDistanceSensor.class, "frontLeftSensor");
        backLeftSensor = hwMap.get(Rev2mDistanceSensor.class, "backLeftSensor");

        backLaser = hwMap.get(Rev2mDistanceSensor.class, "backSensor");
        frontLaser = hwMap.get(Rev2mDistanceSensor.class, "frontSensor");
*/

    }
    public HardwareMap getHwMap() {
        return hwMap;
    }
}