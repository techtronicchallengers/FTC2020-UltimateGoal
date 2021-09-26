package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

@Disabled
@TeleOp(name = "EncoderTest", group = "Test")
public class TestEncoder extends LinearOpMode {

    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double COUNTS_PER_DEGREE = 9.0;

    private static final double COUNTS_PER_SIDE_INCH = 50;

    private static final double MOTOR_SPEED = 0.8;


    // Creating a Rover robot object
    UltimateBot utlimateBot = new UltimateBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        utlimateBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {

            //*Get the position of the motor(s) being tested and print*//
            //Wheel
            telemetry.addData("FR", utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
            telemetry.addData("FL", utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition());
            telemetry.addData("BR", utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition());
            telemetry.addData("BL", utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition());

            telemetry.update();



            //* Controlling the Motors *//
            //Move the motor
            if (gamepad1.x)
            {
                utlimateBot.getChassisAssembly().moveRight(MOTOR_SPEED);
            }
            else if (gamepad1.y)
            {
                encoderSide(MOTOR_SPEED, 12, 10);
            }
            else {
                utlimateBot.getChassisAssembly().stopMoving();
            }

        }
    }

    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            utlimateBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);



            utlimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            utlimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            utlimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            utlimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            utlimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            utlimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (utlimateBot.getChassisAssembly().isBackLeftWheelBusy() && utlimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            utlimateBot.getChassisAssembly().isFrontLeftWheelBusy() && utlimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            utlimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            utlimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }//end of encoderDrive

    public void encoderSide(double speed, double inches, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newBackRightTarget = utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
            newFrontLeftTarget = utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
            newFrontRightTarget = utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);

            utlimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            utlimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            utlimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            utlimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            utlimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            utlimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (utlimateBot.getChassisAssembly().isBackLeftWheelBusy() && utlimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            utlimateBot.getChassisAssembly().isFrontLeftWheelBusy() && utlimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            utlimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            utlimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(250);
    }//end of encoderSide

    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);

            utlimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            utlimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            utlimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            utlimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            utlimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            utlimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            utlimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (utlimateBot.getChassisAssembly().isBackLeftWheelBusy() && utlimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            utlimateBot.getChassisAssembly().isFrontLeftWheelBusy() && utlimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        utlimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        utlimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            utlimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            utlimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(250);
    }//end of encoderTurn
}
