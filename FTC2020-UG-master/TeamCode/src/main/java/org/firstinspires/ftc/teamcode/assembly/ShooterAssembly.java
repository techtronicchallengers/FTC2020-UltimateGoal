package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterAssembly
{
    final public double shootingSpeed = 1.0;
    public RobotHardware robotHardware;

    public ShooterAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void shoot()
    {
        robotHardware.shooter.setPower(shootingSpeed);
        robotHardware.guideWheel.setPower(-0.8);
    }
    public void stopShoot()
    {
        robotHardware.shooter.setPower(0.0);
        robotHardware.guideWheel.setPower(0.0);
    }

    public void moveLift(double speed)
    {
        robotHardware.lift.setPower(speed);
    }
    public void stopLift()
    {
        robotHardware.lift.setPower(0.0);
    }

    public void pushRing()
    {
        robotHardware.pusher.setPosition(0.37);
    }
    public void returnPusher()
    {
        robotHardware.pusher.setPosition(0.48);
    }

    public void openDoor()
    {
        robotHardware.door.setPosition(0.39);
    }
    public void closeDoor()
    {
        robotHardware.door.setPosition(0.52);
    }

    public void changeShooterAng(double position)
    {
        robotHardware.shooterAngleServo.setPosition(position);
    }

    public void powerShotAng()
    {
        robotHardware.shooterAngleServo.setPosition(0.66);
    }
    //0.67

    public void highGoalAng()
    {
        robotHardware.shooterAngleServo.setPosition(0.63);
    }
    //0.64

    public void midGoalAng()
    {
        robotHardware.shooterAngleServo.setPosition(0.74);
    }
    //0.77

    public void autoPowerShotAng()
    {
        robotHardware.shooterAngleServo.setPosition(0.73);
    }
}
