package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.assembly.RobotHardware;

public class ChassisAssembly
{

    public RobotHardware robotHardware;

    public ChassisAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }


    /**
     *-----------------------------------------------------------------------------------
     * WHEEL CONTROLS
     * This is used to move the robot around.  The robot can move in all directions with
     * the use of Mecanum wheels
     * ----------------------------------------------------------------------------------
     */

    /**
     * Moves the robot forward by giving power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    public void moveForward(double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot backwards by giving negative power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    public void moveBackwards(double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the right by giving the left wheels power and the right
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    public void turnRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the left by giving the right wheels power and the left
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    public void turnLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot to the right laterally.  The front left wheel and back right wheel
     * are given negative power and the front right wheel and back left wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    public void moveLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot to the left laterally.  The front right wheel and back left wheel
     * are given negative power and the front left wheel and back right wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    public void moveRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    public void diagonalForwardLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(0);
        robotHardware.frontRightWheel.setPower(0);
        robotHardware.backRightWheel.setPower(speed);
    }
    public void diagonalForwardRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(0);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(0);
    }
    public void diagonalBackwardsLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(0);
        robotHardware.frontRightWheel.setPower(0);
        robotHardware.backRightWheel.setPower(-speed);
    }
    public void diagonalBackwardsRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(0);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(0);
    }

    /**
     * Stops the robot by setting the power of all wheels to 0
     */
    public void stopMoving()
    {
        robotHardware.frontLeftWheel.setPower(0);
        robotHardware.frontRightWheel.setPower(0);
        robotHardware.backLeftWheel.setPower(0);
        robotHardware.backRightWheel.setPower(0);
    }



    /**
     *-----------------------------------------------------------------------------------
     * ENCODER WHEEL CONTROLS
     * This is used during the autonomous mode to ensure that the robot moves to accurate
     * distances.
     * There are many different methods that are used when running with encoder.
     * ----------------------------------------------------------------------------------
     */

    /**
     * This method changes the mode the wheels are running in to passed run mode.
     * Some such run modes include RUN_USING_ENCODER and RUN_TO_POSITION, etc.
     * @param mode which the wheels should be set to running in
     */
    public void setMode(DcMotor.RunMode mode)
    {
        robotHardware.backLeftWheel.setMode(mode);
        robotHardware.backRightWheel.setMode(mode);
        robotHardware.frontLeftWheel.setMode(mode);
        robotHardware.frontRightWheel.setMode(mode);
    }

    /**
     * The follow four methods are used to individually set the power for each wheel
     * This is needed when using the encoder
     * @param speed the power that the motor will be set to
     */
    public void setBackLeftWheelPower(double speed) {robotHardware.backLeftWheel.setPower(speed);}
    public void setBackRightWheelPower(double speed) {robotHardware.backRightWheel.setPower(speed);}
    public void setFrontLeftWheelPower(double speed) {robotHardware.frontLeftWheel.setPower(speed);}
    public void setFrontRightWheelPower(double speed) { robotHardware.frontRightWheel.setPower(speed); }


    /**
     * The following four methods test each wheel if they are busy in that the wheel's
     * motor is powered and active
     * @return whether or not the wheel is busy where true means it is busy and false
     * means it is not busy
     */
    public boolean isBackLeftWheelBusy()
    {
        return robotHardware.backLeftWheel.isBusy();
    }
    public boolean isBackRightWheelBusy()
    {
        return robotHardware.backRightWheel.isBusy();
    }
    public boolean isFrontLeftWheelBusy()
    {
        return robotHardware.frontLeftWheel.isBusy();
    }
    public boolean isFrontRightWheelBusy()
    {
        return robotHardware.frontRightWheel.isBusy();
    }


    /**
     * Resets the encoders of all the wheels' motors and changes them to encoder mode
     */
    public void changeToEncoderMode()
    {
        robotHardware.backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * The following four methods gets the current position of the wheels' motors
     * @return an integer value representing the current position of the wheel's motor
     */
    public int getBackRightWheelCurrentPosition() {return robotHardware.backRightWheel.getCurrentPosition();}
    public int getBackLeftWheelCurrentPosition() {return robotHardware.backLeftWheel.getCurrentPosition();}
    public int getFrontRightWheelCurrentPosition() {return robotHardware.frontRightWheel.getCurrentPosition();}
    public int getFrontLeftWheelCurrentPosition() {return robotHardware.frontLeftWheel.getCurrentPosition();}


    /**
     * The following four methods sets the target position to which each wheel's motor should
     * run to.
     */
    public void setBackLeftWheelTargetPosition(int position) {robotHardware.backLeftWheel.setTargetPosition(position);}
    public void setBackRightWheelTargetPosition(int position) {robotHardware.backRightWheel.setTargetPosition(position);}
    public void setFrontLeftWheelTargetPosition(int position) {robotHardware.frontLeftWheel.setTargetPosition(position);}
    public void setFrontRightWeelTargetPosition(int position) {robotHardware.frontRightWheel.setTargetPosition(position);}

}
