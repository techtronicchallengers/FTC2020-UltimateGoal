package org.firstinspires.ftc.teamcode.assembly;

public class IntakeAssembly
{
    final public double intakeSpeed = 1.0;
    public RobotHardware robotHardware;

    public IntakeAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void intake()
    {
        robotHardware.intaker.setPower(intakeSpeed);
    }
    public void outake()
    {
        robotHardware.intaker.setPower(-intakeSpeed);
    }

    public void stopIntake()
    {
        robotHardware.intaker.setPower(0.0);
    }

    public void guardUp()
    {
        robotHardware.guard.setPosition(0.8);
    }

    public void guardDown()
    {
        robotHardware.guard.setPosition(0.05);
    }
}
