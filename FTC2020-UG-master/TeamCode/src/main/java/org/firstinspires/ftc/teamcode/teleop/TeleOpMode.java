package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

import java.util.Locale;

@TeleOp(name = "TeleOpMode", group = "Qualifier")
public class TeleOpMode extends LinearOpMode
{
    Orientation angles;
    Acceleration gravity;

    private double   WHEEL_SPEED = 1.0;
    private static double INTAKE_SPEED = 1.0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_DEGREE = 9;
    final double distanceBetweenSensors = 11.5;

    double shooterAngle = 0.85;
    double angleToStraight = 0;

    int pushedRings = 0;

    boolean isIntaking = false;
    boolean intakePressed = false;
    boolean notPowerAng = true;


    //Creating a Rover robot object
    UltimateBot ultimateBot = new UltimateBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        ultimateBot.initRobot(hardwareMap);
        ultimateBot.getShooterAssembly().returnPusher();
        ultimateBot.getShooterAssembly().closeDoor();
        ultimateBot.getShooterAssembly().highGoalAng();
        ultimateBot.getWobbleAssembly().openGripper();
        ultimateBot.getIntakeAssembly().guardUp();

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

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        while (opModeIsActive())
        {
            //Gamepad 1 Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;

            double armMovement = gamepad1.right_stick_y;

            boolean intake = gamepad1.x;
            boolean shoot = gamepad1 .b;

            boolean gripClose = gamepad1.a;
            boolean gripOpen = gamepad1.y;

            boolean highGoal = gamepad1.dpad_up;
            boolean powerShot = gamepad1.dpad_down;

            //Gamepad 2 Controls
            boolean midGoal = gamepad2.a;
            boolean outake = gamepad2.x;
            boolean stopOutake = gamepad2.b;

            boolean shooterUp = gamepad2.right_bumper;
            boolean shooterDown = gamepad2.left_bumper;

            boolean straightenRobot = gamepad2.dpad_right;
            boolean turnToGoal = gamepad2.dpad_left;

            boolean guardDown = gamepad2.dpad_down;
            boolean guardUp = gamepad2.dpad_up;

            boolean shootRing = gamepad2.y;

            telemetry.addData("front right sensor", "%.2f in", ultimateBot.getRobotHardware().rightFrontSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("back right sensor", "%.2f in", ultimateBot.getRobotHardware().rightBackSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("back sensor", "%.2f in", ultimateBot.getRobotHardware().backSensor.getDistance(DistanceUnit.INCH));

            telemetry.update();

            //Movement
            if (drive < 0) {
                ultimateBot.getChassisAssembly().moveForward(Math.abs(drive));
            }
            else if (drive > 0){
                ultimateBot.getChassisAssembly().moveBackwards(Math.abs(drive));
            }
            //turn right
            else if (turn > 0) {
                ultimateBot.getChassisAssembly().turnRight(Math.abs(turn));
            }
            //turn left
            else if (turn < 0) {
                ultimateBot.getChassisAssembly().turnLeft(Math.abs(turn));
            }
            //side right
            else if (sideRight > 0) {
                ultimateBot.getChassisAssembly().moveRight(sideRight);
            }
            //side left
            else if (sideLeft > 0) {
                ultimateBot.getChassisAssembly().moveLeft(sideLeft);
            }
         /*   else if(diagonalFrontRight > 0)
            {
                ultimateBot.getChassisAssembly().diagonalForwardRight(WHEEL_SPEED);
            }
            else if(diagonalFrontLeft < 0)
            {
                ultimateBot.getChassisAssembly().diagonalForwardLeft(WHEEL_SPEED);
            }
            else if(diagonalBackRight > 0)
            {
                ultimateBot.getChassisAssembly().diagonalBackwardsRight(WHEEL_SPEED);
            }
            else if(diagonalBackLeft < 0)
            {
                ultimateBot.getChassisAssembly().diagonalBackwardsLeft(WHEEL_SPEED);
            }
          */
            //stop moving
            else
            {
                ultimateBot.getChassisAssembly().stopMoving();
            }

            if(intake == false && intakePressed == true)
            {
                intakePressed = false;
            }
            if(intake == true && isIntaking == false && intakePressed == false)
            {
                ultimateBot.getIntakeAssembly().intake();
                isIntaking = true;
                intakePressed = true;
            }
            if(intake == true && isIntaking == true && intakePressed == false)
            {
                ultimateBot.getIntakeAssembly().stopIntake();
                isIntaking = false;
                intakePressed = true;
            }

            if(shoot == true && notPowerAng == true)
            {
                ultimateBot.getChassisAssembly().stopMoving();
                ultimateBot.getIntakeAssembly().stopIntake();

              //  turnToGoal(1.0);
              //  encoderTurn(1.0, 12.036, 5);

                ultimateBot.getShooterAssembly().shoot();

                while(opModeIsActive() && ultimateBot.getRobotHardware().topTouch.isPressed() == false)
                {
                    ultimateBot.getShooterAssembly().moveLift(1.0);
                }
                ultimateBot.getShooterAssembly().stopLift();
                ultimateBot.getShooterAssembly().openDoor();
                sleep(500);

                for(int i = 0; i < 4; i++)
                {
                    ultimateBot.getShooterAssembly().pushRing();
                    sleep(400);
                    ultimateBot.getShooterAssembly().returnPusher();
                    sleep(600);
                }

                while(opModeIsActive() && ultimateBot.getRobotHardware().bottomTouch.isPressed() == false)
                {
                    ultimateBot.getShooterAssembly().moveLift(-0.5);
                }
                ultimateBot.getShooterAssembly().stopLift();
                ultimateBot.getShooterAssembly().closeDoor();

                ultimateBot.getShooterAssembly().stopShoot();
            }

            else if(shoot == true && notPowerAng == false)
            {
                ultimateBot.getChassisAssembly().stopMoving();
                ultimateBot.getIntakeAssembly().stopIntake();
                ultimateBot.getShooterAssembly().shoot();

                while(opModeIsActive() && ultimateBot.getRobotHardware().topTouch.isPressed() == false)
                {
                    ultimateBot.getShooterAssembly().moveLift(1.0);
                }
                ultimateBot.getShooterAssembly().stopLift();
                ultimateBot.getShooterAssembly().openDoor();
                sleep(500);

                ultimateBot.getShooterAssembly().pushRing();
                sleep(400);
                ultimateBot.getShooterAssembly().returnPusher();
                sleep(400);

                pushedRings = pushedRings + 1;

                if(pushedRings == 3)
                {
                    while(opModeIsActive() && ultimateBot.getRobotHardware().bottomTouch.isPressed() == false)
                    {
                        ultimateBot.getShooterAssembly().moveLift(-0.5);
                    }
                    ultimateBot.getShooterAssembly().stopLift();
                    ultimateBot.getShooterAssembly().closeDoor();

                    ultimateBot.getShooterAssembly().stopShoot();

                    pushedRings = 0;
                }
            }

            if(powerShot == true)
            {
                ultimateBot.getShooterAssembly().powerShotAng();
                notPowerAng = false;
            }

            if(highGoal == true)
            {
                ultimateBot.getShooterAssembly().highGoalAng();
                notPowerAng = true;
            }

            if(gripOpen == true)
            {
                ultimateBot.getWobbleAssembly().openGripper();
                sleep(500);

                ultimateBot.getChassisAssembly().stopMoving();
                while(opModeIsActive() &&  ultimateBot.getRobotHardware().armReturn.isPressed() == false)
                {
                    ultimateBot.getWobbleAssembly().moveArm(1.0);
                }
                ultimateBot.getWobbleAssembly().stopArm();
            }

            if(gripClose == true)
            {
                ultimateBot.getWobbleAssembly().closeGripper();
            }

            if(armMovement > 0 &&  ultimateBot.getRobotHardware().grabTouch.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(-0.75);
            }
            else if(armMovement < 0)
            {
                ultimateBot.getChassisAssembly().stopMoving();
                while(opModeIsActive() &&  ultimateBot.getRobotHardware().armReturn.isPressed() == false)
                {
                    ultimateBot.getWobbleAssembly().moveArm(0.75);
                }
                ultimateBot.getWobbleAssembly().stopArm();
            }
            else
            {
                ultimateBot.getWobbleAssembly().stopArm();
            }

            //Gamepad 2

            if(midGoal == true)
            {
                ultimateBot.getShooterAssembly().midGoalAng();
                notPowerAng = true;
            }

            if(outake == true)
            {
                ultimateBot.getIntakeAssembly().outake();
            }

            if(stopOutake == true)
            {
                ultimateBot.getIntakeAssembly().stopIntake();
            }
/*
            //up pos = 0.35, down pos = 0.9
            if(shooterUp == true)
            {
                ultimateBot.getShooterAssembly().changeShooterAng(shooterAngle);
                sleep(100);
                shooterAngle = shooterAngle - 0.01;
                telemetry.addData("Shooter Angle", shooterAngle);
                telemetry.update();
            }

            if(shooterDown == true)
            {
                ultimateBot.getShooterAssembly().changeShooterAng(shooterAngle);
                sleep(100);
                shooterAngle = shooterAngle + 0.01;
                telemetry.addData("Shooter Angle", shooterAngle);
                telemetry.update();
            }
*/
            if(guardUp == true)
            {
                ultimateBot.getIntakeAssembly().guardUp();
            }
            if(guardDown == true)
            {
                ultimateBot.getIntakeAssembly().guardDown();
            }

            /*
            if(straightenRobot == true)
            {
                angleToStraight = Math.atan(distanceBetweenSensors/(ultimateBot.getRobotHardware().rightFrontSensor.getDistance(DistanceUnit.INCH)
                        - ultimateBot.getRobotHardware().rightBackSensor.getDistance(DistanceUnit.INCH)));
                angleToStraight = 90 - (angleToStraight * 180/Math.PI);

                telemetry.addData("Angle to turn ", angleToStraight);
                telemetry.update();
                sleep(2000);
                encoderTurn(1.0, -angleToStraight, 5.0);
            }

            if(turnToGoal == true)
            {
                angleToStraight = Math.atan(distanceBetweenSensors/(ultimateBot.getRobotHardware().rightFrontSensor.getDistance(DistanceUnit.INCH)
                        - ultimateBot.getRobotHardware().rightBackSensor.getDistance(DistanceUnit.INCH)));

                double x_distance = (Math.sin(angleToStraight) * 180/Math.PI) * (ultimateBot.getRobotHardware().rightFrontSensor.getDistance(DistanceUnit.INCH)
                        + ultimateBot.getRobotHardware().rightBackSensor.getDistance(DistanceUnit.INCH));
                x_distance = x_distance/2;

                double y_distance = (Math.sin(angleToStraight)* 180/Math.PI) * (ultimateBot.getRobotHardware().backSensor.getDistance(DistanceUnit.INCH));

                double angleToGoal = Math.atan((60-x_distance)/(144-y_distance)) * 180/Math.PI;
                angleToGoal = (angleToStraight * 180/Math.PI) + angleToGoal - 90;

                telemetry.addData("x value ", x_distance);
                telemetry.addData("y value ", y_distance);
                telemetry.addData("Angle to turn ", angleToGoal);
                telemetry.update();
                //sleep(2000);

                encoderTurn(1.0, angleToGoal, 5.0);
            }

             */
        }
    }

    public void turnToGoal(double speed)
    {
        double angleToStraight = Math.atan(distanceBetweenSensors/(ultimateBot.getRobotHardware().rightFrontSensor.getDistance(DistanceUnit.INCH)
                - ultimateBot.getRobotHardware().rightBackSensor.getDistance(DistanceUnit.INCH)));

        double x_distance = (Math.sin(angleToStraight) * 180/Math.PI) * (ultimateBot.getRobotHardware().rightFrontSensor.getDistance(DistanceUnit.INCH)
                + ultimateBot.getRobotHardware().rightBackSensor.getDistance(DistanceUnit.INCH));
        x_distance = x_distance/2;

        double y_distance = (Math.sin(angleToStraight)* 180/Math.PI) * (ultimateBot.getRobotHardware().backSensor.getDistance(DistanceUnit.INCH));

        double angleToGoal = Math.atan((60-x_distance)/(144-y_distance)) * 180/Math.PI;
        angleToGoal = (angleToStraight * 180/Math.PI) + angleToGoal - 90;

        telemetry.addData("x value ", x_distance);
        telemetry.addData("y value ", y_distance);
        telemetry.addData("Angle to turn ", angleToGoal);
        telemetry.update();

        encoderTurn(speed, angleToGoal, 5.0);
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

    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
    public void encoderDrive(double speed, double inches, double timeoutS)
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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

    void readAngle()
    {
        angles   = ultimateBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}