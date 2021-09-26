package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

@Disabled
@TeleOp(name = "LaserDistanceTest", group = "Test")
public class TestLaserSensors extends LinearOpMode
{
    //Creating a Rover robot object
    UltimateBot utlimateBot = new UltimateBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        utlimateBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {
            telemetry.addData("Front Laser", utlimateBot.getNavigation().frontDistance());
            telemetry.addData("Back Laser", utlimateBot.getNavigation().backDistance());
            telemetry.update();
        }
    }
}
