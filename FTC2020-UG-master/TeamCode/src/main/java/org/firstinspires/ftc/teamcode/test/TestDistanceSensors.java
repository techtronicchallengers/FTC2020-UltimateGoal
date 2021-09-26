package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

@Disabled
@TeleOp(name = "DistanceSensorsTest", group = "Test")
public class TestDistanceSensors extends LinearOpMode
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
            telemetry.addData("Front Right", ultimateBot.getNavigation().frontRightDistance());
            telemetry.addData("Back Right", ultimateBot.getNavigation().backRightDistance());
            telemetry.addData("Front Left", ultimateBot.getNavigation().frontLeftDistance());
            telemetry.addData("Back Left", ultimateBot.getNavigation().backLeftDistance());

            telemetry.addData("Front", ultimateBot.getNavigation().frontDistance());
            telemetry.addData("Back", ultimateBot.getNavigation().backDistance());

            telemetry.addData("Right Angle", ultimateBot.getNavigation().rightAngle());
            telemetry.addData("Left Angle", ultimateBot.getNavigation().leftAngle());
            telemetry.update();
        }
    }
}