package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.assembly.ChassisAssembly;
import org.firstinspires.ftc.teamcode.assembly.SensorNavigation;
import org.firstinspires.ftc.teamcode.assembly.VisualCortex;
import org.firstinspires.ftc.teamcode.assembly.UltimateBot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.Locale;



@Autonomous(name = "RRHWP", group = "League")
public class RRHWP extends LinearOpMode
{

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SIDE_INCH = 50;
    final double COUNTS_PER_DEGREE = 6.59;
    double PUSHER_POS = 0;

    //target constants
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Servo Positions


    //Movement Constants
    final double WHEEL_SPEED = 1;
    final double SIDE_SHIFT = 6;

    //Creating a  robot object
    UltimateBot ultimateBot = new UltimateBot();
    VisualCortex vcortex = null;
    SensorNavigation nav = null;
    ChassisAssembly chassis = null;;
    List<VuforiaTrackable> allTrackables = null;


    //Time
    ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera webCam;
    RingDeterminationPipeline pipeline;
    int numRings;

    @Override public void runOpMode()
    {
        //Intialize Robot
        ultimateBot.initRobot(hardwareMap);
        nav = ultimateBot.getNavigation();
        chassis = ultimateBot.getChassisAssembly();

        //Gyro Setup/Initialization
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "imuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        ultimateBot.getRobotHardware().imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();

        //Initilize Vuforia and tensor flow
        /* ultimateBot.initializeVuforiaAndTensorFlow();  //uncomment this when vuforia and tflod is needed
         ultimateBot.loadVuforiaTrackables();
         vcortex = ultimateBot.getVisualCortex();
        allTrackables = vcortex.getAllTrackables();
        */
        // Initialize Easy CV for ring detection using webcam
        int cameraMonitorViewId = ultimateBot.getRobotHardware().getHwMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam( ultimateBot.getRobotHardware().getHwMap().get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline(telemetry);
        webCam.setPipeline(pipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });


        //Wait for Start
        telemetry.addData("Waiting for start", "");

        ultimateBot.getShooterAssembly().changeShooterAng(0.62);
        ultimateBot.getShooterAssembly().returnPusher();
        ultimateBot.getWobbleAssembly().closeGripper();
        ultimateBot.getShooterAssembly().closeDoor();

        /* use this for using TF for ring detection
        ArrayList<Integer> stackArrayList=new ArrayList<Integer>();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Scanning stack ", "..Waiting for Start");
            telemetry.update();
            int stack = vcortex.checkStarterStack(3);
            stackArrayList.add(new Integer(stack));
            // if there are more than 10 scans, throw away the first five scans from the list
            if(stackArrayList.size() > 10)
            {
                stackArrayList.subList(0, 5).clear();
            }
        }*/


        waitForStart();
        ultimateBot.getIntakeAssembly().guardDown();
        numRings = pipeline.getNumRings();
        telemetry.addData("numRings", numRings);
        telemetry.update();

        rampSpeedEncoderDrive(0.7, 40, 7);
        turnToAngle(1, 26, 3);

        ultimateBot.getShooterAssembly().shoot();
        ultimateBot.getIntakeAssembly().stopIntake();
        while (opModeIsActive() && ultimateBot.getRobotHardware().topTouch.isPressed() == false) {
            ultimateBot.getShooterAssembly().moveLift(1.0);
        }
        ultimateBot.getShooterAssembly().stopLift();
        ultimateBot.getShooterAssembly().openDoor();
        sleep(500);

        for(int i = 0; i < 3; i++)
        {
            ultimateBot.getShooterAssembly().pushRing();
            sleep(400);
            ultimateBot.getShooterAssembly().returnPusher();
            sleep(400);
        }

        while (opModeIsActive() && ultimateBot.getRobotHardware().bottomTouch.isPressed() == false) {
            ultimateBot.getShooterAssembly().moveLift(-0.5);
        }
        ultimateBot.getShooterAssembly().stopLift();
        ultimateBot.getShooterAssembly().closeDoor();

        ultimateBot.getShooterAssembly().stopShoot();

        sleep(10000);
        rampSpeedEncoderDrive(1, 20, 3);



/*
        if(numRings == 0)
        {
            turnToAngle(1, 0, 2 );
            rampSpeedEncoderDrive(1, 20, 7);
            turnToAngle(1, 90, 4);
            rampSpeedEncoderDrive(1, 9, 4);

            while (opModeIsActive() && ultimateBot.getRobotHardware().grabTouch.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(-0.5);
            }
            ultimateBot.getWobbleAssembly().stopArm();

            ultimateBot.getWobbleAssembly().openGripper();
            sleep(500);

            while (opModeIsActive() && ultimateBot.getRobotHardware().armReturn.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(1.0);
            }
            ultimateBot.getWobbleAssembly().stopArm();

            turnToAngle(1, 95, 1);
            rampSpeedEncoderDrive(1, 45, 5);
        }

        if(numRings == 1)
        {
            turnToAngle(1, 0, 2 );
            rampSpeedEncoderDrive(1, 40, 4);
            turnToAngle(1, 90, 4);
            rampSpeedEncoderDrive(1, 34, 4 );

            while (opModeIsActive() && ultimateBot.getRobotHardware().grabTouch.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(-0.5);
            }
            ultimateBot.getWobbleAssembly().stopArm();

            ultimateBot.getWobbleAssembly().openGripper();
            sleep(500);

            while (opModeIsActive() && ultimateBot.getRobotHardware().armReturn.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(1.0);
            }

            ultimateBot.getWobbleAssembly().stopArm();

            rampSpeedEncoderDrive(1, 15, 4);
            turnToAngle(1, 0, 4);
            rampSpeedEncoderDrive(1, -28, 4);
        }

        if(numRings == 4)
        {
            turnToAngle(1, 0, 2);
            rampSpeedEncoderDrive(1, 65, 7);
            turnToAngle(1,  90, 3);
            rampSpeedEncoderDrive(1, 9, 4);
            while (opModeIsActive() && ultimateBot.getRobotHardware().grabTouch.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(-0.5);
            }
            ultimateBot.getWobbleAssembly().stopArm();

            ultimateBot.getWobbleAssembly().openGripper();
            sleep(500);

            while (opModeIsActive() && ultimateBot.getRobotHardware().armReturn.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(1.0);
            }

            ultimateBot.getWobbleAssembly().stopArm();

            rampSpeedEncoderDrive(1, 45, 5);
            turnToAngle(1, 0, 4);
            rampSpeedEncoderDrive(1, -46, 5);
        }
*/
    }


    public void turnToAngle(double speed, double desiredAngle, int numLoops)
    {
        double currentAngle = angles.firstAngle;
        double angleToTurn;

        for(int i = 0; i < numLoops; i++)
        {
            readAngle();
            currentAngle = angles.firstAngle;
            angleToTurn = desiredAngle - currentAngle;

            if(Math.abs(angleToTurn) > 180)
            {
                double newAngleToTurn = 360 - Math.abs(angleToTurn);
                if(angleToTurn > 0)
                {
                    newAngleToTurn = -Math.abs(newAngleToTurn);
                    angleToTurn = newAngleToTurn;
                }
                else
                {
                    newAngleToTurn = Math.abs(newAngleToTurn);
                    angleToTurn = newAngleToTurn;
                }
            }

            encoderTurn(speed, angleToTurn, 5);
        }

        telemetry.addData("current angle", currentAngle);
        telemetry.addData("desired angle", desiredAngle);
        telemetry.update();
    }


    void readAngle()
    {
        angles   = ultimateBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = ultimateBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = ultimateBot.getRobotHardware().imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return ultimateBot.getRobotHardware().imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return ultimateBot.getRobotHardware().imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void straightenLeft(double timeOut)
    {
        double angle = -ultimateBot.getNavigation().leftAngle();
        ElapsedTime senseTime = new ElapsedTime();
        while(Math.abs(angle) > 15 && opModeIsActive() && senseTime.seconds() < timeOut)
        {
            angle = -ultimateBot.getNavigation().leftAngle();
        }
        if(angle > 50)
        {
            angle = 0;
        }
        telemetry.addData("Angle", angle);
        telemetry.update();

        if(Math.abs(angle) > 5)
        {
            encoderTurn(WHEEL_SPEED, angle, 5);
        }
    }


    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
    public void rampSpeedEncoderDrive(double speed, double inches, double timeoutS)
    {
        double setSpeed = speed;
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            ultimateBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);


            ultimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            ultimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            ultimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            ultimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            speed = 0.1;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (ultimateBot.getChassisAssembly().isBackLeftWheelBusy() && ultimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            ultimateBot.getChassisAssembly().isFrontLeftWheelBusy() && ultimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                while((Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/4)
                        && Math.abs(speed) < Math.abs(setSpeed)))
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while((Math.abs(speed) < Math.abs(setSpeed))
                        && Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while(Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                }
                while((Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) > Math.abs(newFrontLeftTarget/2)) && speed > 0.1)
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed -  0.01;
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            ultimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of rampSpeedEncoderDrive

    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            ultimateBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);



            ultimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            ultimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            ultimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            ultimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (ultimateBot.getChassisAssembly().isBackLeftWheelBusy() && ultimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            ultimateBot.getChassisAssembly().isFrontLeftWheelBusy() && ultimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            ultimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }//end of encoderDrive

    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);

            ultimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            ultimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            ultimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            ultimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (ultimateBot.getChassisAssembly().isBackLeftWheelBusy() && ultimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            ultimateBot.getChassisAssembly().isFrontLeftWheelBusy() && ultimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            ultimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderTurn

    public void encoderSide(double speed, double inches, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = chassis.getBackLeftWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
            newBackRightTarget = chassis.getBackRightWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newFrontLeftTarget = chassis.getFrontLeftWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newFrontRightTarget = chassis.getFrontRightWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);

            chassis.setBackLeftWheelTargetPosition(newBackLeftTarget);
            chassis.setBackRightWheelTargetPosition(newBackRightTarget);
            chassis.setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            chassis.setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            chassis.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            chassis.setBackLeftWheelPower(Math.abs(speed));
            chassis.setBackRightWheelPower(Math.abs(speed));
            chassis.setFrontLeftWheelPower(Math.abs(speed));
            chassis.setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.isBackLeftWheelBusy() && chassis.isBackRightWheelBusy() &&
                            chassis.isFrontLeftWheelBusy() && chassis.isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        chassis.getBackLeftWheelCurrentPosition(),
                        chassis.getBackRightWheelCurrentPosition(),
                        chassis.getFrontLeftWheelCurrentPosition(),
                        chassis.getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            chassis.stopMoving();

            // Turn off RUN_TO_POSITION
            chassis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(250);
    }//end of encoderSide

    /*
    public int  getZoneUsingTF(ArrayList<Integer> stack, int numOfScans)
    {
        int totalTries = stack.size();
        telemetry.addData("totalTries ", totalTries);
        int numRings = 0;
        int singleCount= 0;
        int noneCount = 0;
        int quadCount = 0;

        int scansCount = 0;
        if(totalTries < numOfScans)
        {
            scansCount = totalTries;
        }
        else
        {
            scansCount= numOfScans;
        }

        for(int i= totalTries- 1; i >= totalTries - scansCount ; i--)
        {
            if(stack.get(i) ==0) { noneCount++;  }
            else if(stack.get(i) ==1)  { singleCount++;   }
            else if(stack.get(i) ==4)  { quadCount++;    }

        }
        telemetry.addData("noneCount ", noneCount);
        telemetry.addData("singleCount ", singleCount);
        telemetry.addData("quadCount ", quadCount);
        telemetry.update();
        if( noneCount >= singleCount && noneCount >= quadCount) {
            telemetry.addData("No Ring Score  is " + noneCount + " out of last " + scansCount , "  Zone A") ;
            numRings  =4;
        }
        else if (singleCount >= noneCount && singleCount >= quadCount) {
            telemetry.addData("Single Ring Score  is " + singleCount + " out of last " + scansCount , " Zone B") ;
            numRings =1;
        }
        else {
            telemetry.addData("Quad Rings Score  is " + quadCount + " out of last " + scansCount , "Zone C") ;
            numRings = 1;
        }
        telemetry.update();
        return numRings;

    }
    */

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {

        static Telemetry telemetry;

        public RingDeterminationPipeline(Telemetry tele) {
            telemetry = tele;
        }

        int numRings;

        @Override
        public Mat processFrame(Mat input)
        {

            int startRow = 0;
            int endRow = 0;
            int startColumn = 0;
            int endColumn = 0;

            double[] pixel;
            Size size = input.size();
            double height = size.height;
            double width = size.width;

            double redValue = 0;
            double greenValue = 0;
            double blueValue = 0;
            for (int i=100; i < width; i++)
            {
                for (int j=0; j < height; j++)
                {
                    pixel = input.get(i,j);
                    if (pixel != null && pixel.length > 0)
                    {
                        redValue = pixel[0];
                        greenValue = pixel[1];
                        blueValue= pixel[2];
                        if (redValue > 150 && greenValue > 100 && blueValue < 50) {
                            if ( startRow <= 0)
                                startRow = i;
                            if (startColumn <= 0)
                                startColumn = j;

                            if (endRow < i)
                                endRow = i;

                            if (endColumn < j)
                                endColumn = j;
                        }
                    }
                    else
                    {
                        //telemetry.addData("Pixel Null at",  i );
                        //telemetry.addData(" ",  j );
                        //telemetry.update();
                    }
                }
            }

            double ringWidth = endRow - startRow;
            double ringHeight = endColumn - startColumn;

            double ratio =  0;
            if (ringHeight > 0.0)
                ratio = ringWidth/ringHeight;


            if(ringWidth>70){}
            else if (ringWidth > 20)
                numRings = 4;
            else if (ringWidth > 0.1 )
                numRings = 1;
            else
                numRings = 0;

            telemetry.addData("Start Row ",  startRow );
            telemetry.addData("End Row ",  endRow ); //Display it on telemetry
            telemetry.addData("Start Column ",  startColumn );
            telemetry.addData("End Column ",  endColumn );
            telemetry.addData("Ratio ",  ratio );
            telemetry.addData("Width:", ringWidth);
            telemetry.addData("Height:", ringHeight);
            telemetry.addData("Number of Rings ",  numRings );
            telemetry.update();

            return input;
        }


        public int getNumRings()
        {
            return numRings;
        }
    }
}
