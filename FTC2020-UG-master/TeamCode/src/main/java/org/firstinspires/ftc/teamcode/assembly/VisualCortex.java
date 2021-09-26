package org.firstinspires.ftc.teamcode.assembly;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.util.Log;

public class VisualCortex {
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AVzGb/f/////AAABmfntLKMN70R8rkNraBi7YAeLTEtl6FX2+8VmWnsm51OfCxTdkx3AVb8HLaYrfQqmY/CgeLg4awajYcLBetMCGNEhyU2kW5oe8eZJCdeUny3KqtdNwO6FDhT8pySyYlyDHGOP1/nnPIrw1+C0IcHEv19TfLO2M5g4LRIdv2FO43xWzxySJdE0qIvXCT+VhEYbIcnpM5isRrJ6PwqPJzZrPUuYgEiqi3FODKKGBtA7owTDrKel4GIxEXIWG43LSPdkH0Mr0OAKmkVhyrXPKaczeLNXBVX5BXSxCHy2b6Zld5GGWz4xBSdKuZboPReLwMq/nxeMVgXZpCvCG9NOy5KqdeUA2qvsQ8DqYb9RA8vEcwU+";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    // Class Members
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private VuforiaLocalizer vuforia = null;
    private VuforiaLocalizer.Parameters parameters = null;
    private TFObjectDetector tfod;

    //Time
    ElapsedTime runtime = new ElapsedTime();

    public static List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    RobotHardware robotHardware;

    public VisualCortex(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void loadTrackables() {


        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");


        allTrackables.addAll(targetsUltimateGoal);

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsUltimateGoal.activate();

    }


    public void stopTracking( VuforiaTrackables targetsUltimateGoal )
    {
        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }
    public  TFObjectDetector getTfod()
    {
        return tfod;
    }
    public  List<VuforiaTrackable> getAllTrackables()
    {
        return allTrackables;
    }

    public void initVuforia() {
        int cameraMonitorViewId = robotHardware.getHwMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robotHardware.getHwMap().appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robotHardware.webcam;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = robotHardware.getHwMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robotHardware.getHwMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        if (tfod != null) {
            tfod.activate();
        }

    }
    public boolean isTargetVisible(VuforiaTrackable target,double runForSeconds) {

        ElapsedTime runtime = new ElapsedTime();
        boolean targetVisible = false;
        while (runtime.seconds() < runForSeconds) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (trackable.getName().equalsIgnoreCase(target.getName())) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        targetVisible = true;
                        break;
                    }
                }
            }
            if(targetVisible)
            {
                break;
            }
        }
        return targetVisible;

    }
    public OpenGLMatrix getCurrentLocationFromTarget(VuforiaTrackable target,double runForSeconds) {
        OpenGLMatrix lastLocation = null;
        ElapsedTime runtime = new ElapsedTime();
        boolean targetVisible = false;
        while (runtime.seconds() < runForSeconds) {
            for (VuforiaTrackable trackable : allTrackables) {
                {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()
                            && target.getName().equalsIgnoreCase(trackable.getName())) {
                        targetVisible = true;
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                    if(targetVisible)
                    {
                        break;
                    }
                }
            }

        }
        return lastLocation;

    }
    public VectorF getTranslation(OpenGLMatrix loc ) {
        VectorF translation = null;
        // express position (translation) of robot in inches.
        if(loc !=null)
        {
            translation = loc.getTranslation();
        }
        return translation;

    }
    public Orientation getOrientation(OpenGLMatrix loc) {
        Orientation rotation = null;
        if(loc !=null) {
            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(loc, EXTRINSIC, XYZ, DEGREES);
        }
        return rotation;

    }
    public int checkStarterStack(double runForSeconds) {
        ElapsedTime runtime = new ElapsedTime();
        int stack = 0;
        if (tfod != null) {
            while (runtime.seconds() < runForSeconds) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                       if (recognition.getLabel().equalsIgnoreCase("Single")) {
                            stack = 1;
                            break;
                        } else if (recognition.getLabel().equalsIgnoreCase("Quad"))
                        {
                            stack = 4;
                            break;
                        }
                    }

                }
            }
        }

        return stack;
    }
}