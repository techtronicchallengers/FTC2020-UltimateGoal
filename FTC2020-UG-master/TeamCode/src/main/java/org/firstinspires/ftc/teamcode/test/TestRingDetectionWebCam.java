package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.assembly.UltimateBot;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "TestRingDetectionWebCam", group = "Qualifier")
public class TestRingDetectionWebCam extends LinearOpMode
{
    OpenCvCamera webCam;
    RingDeterminationPipeline pipeline;
    UltimateBot ultimateBot = new UltimateBot();

    @Override
    public void runOpMode()
    {

        ultimateBot.initRobot(hardwareMap);

        int cameraMonitorViewId = ultimateBot.getRobotHardware().getHwMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline(telemetry);
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
 //       webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("numRings", pipeline.getNumRings());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(5);
        }
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {

        static Telemetry telemetry;

        public RingDeterminationPipeline(Telemetry tele) {
            telemetry = tele;
        }

        int numRings;

        @Override
        public  Mat processFrame(Mat input)
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
                        if (redValue > 150 && greenValue > 50 && blueValue < 50) {
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


            if (ratio > 0.7)
                numRings = 4;
            else if (ratio > 0.1 )
                numRings = 1;
            else
                numRings = 0;

            telemetry.addData("Start Row ",  startRow );
            telemetry.addData("End Row ",  endRow ); //Display it on telemetry
            telemetry.addData("Start Column ",  startColumn );
            telemetry.addData("End Column ",  endColumn );
            telemetry.addData("Ratio ",  ratio );
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