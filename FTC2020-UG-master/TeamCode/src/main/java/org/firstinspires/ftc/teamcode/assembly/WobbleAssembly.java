package org.firstinspires.ftc.teamcode.assembly;

public class WobbleAssembly
{
    public RobotHardware robotHardware;

    public WobbleAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void moveArm(double power)
    {
        robotHardware.arm.setPower(power);
    }

    public void stopArm()
    {
        robotHardware.arm.setPower(0.0);
    }

    public void closeGripper()
    {
        robotHardware.gripper.setPosition(0.05);
    }

    public void openGripper()
    {
        robotHardware.gripper.setPosition(1.0);
    }
}
