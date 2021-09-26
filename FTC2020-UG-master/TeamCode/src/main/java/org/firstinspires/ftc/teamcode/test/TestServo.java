package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

@Disabled
@TeleOp(name = "ServoTest", group = "Test")
public class TestServo extends LinearOpMode
{
    //Creating a Rover robot object
    UltimateBot ultimateBot = new UltimateBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        ultimateBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {
            if(gamepad1.x)
            {

            }

        }
    }
}