package org.firstinspires.ftc.teamcode.assembly;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.assembly.RobotHardware;

public class SensorNavigation
{
    RobotHardware robotHardware;

    public SensorNavigation(RobotHardware hardware)
    {
        robotHardware = hardware;
    }


    public double frontRightDistance(){return robotHardware.frontRightSensor.getDistance(DistanceUnit.INCH);}
    public double backRightDistance(){return robotHardware.backRightSensor.getDistance(DistanceUnit.INCH);};

    public double frontLeftDistance(){return robotHardware.frontLeftSensor.getDistance(DistanceUnit.INCH);}
    public double backLeftDistance(){return robotHardware.backLeftSensor.getDistance(DistanceUnit.INCH);}

    public double rightDistance(){return 0.5 * (frontRightDistance() + backRightDistance());};

    public double rightAngle()
    {
        final double rightDistanceBetweenSensors = 12;

        double frontDistance = frontRightDistance() + 0.25;
        double backDistance = backRightDistance();

        if(Double.isNaN(frontDistance) || Double.isNaN((backDistance)) || frontDistance == 0 || backDistance == 0)
        {
            return 10000;
        }

        if(frontDistance > 100 || backDistance > 100)
        {
            return 10000;
        }


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/rightDistanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    public double leftDistance(){return 0.5 * (frontLeftDistance() + backLeftDistance());};

    public double leftAngle()
    {
        final double leftDistanceBetweenSensors = 10;

        double frontDistance = frontLeftDistance();
        double backDistance = backLeftDistance();

        if(Double.isNaN(frontDistance) || Double.isNaN((backDistance)) || frontDistance == 0 || backDistance == 0)
        {
            return 10000;
        }

        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/leftDistanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    public double backDistance() { return robotHardware.backLaser.getDistance(DistanceUnit.INCH);}
    public double frontDistance() { return robotHardware.frontLaser.getDistance(DistanceUnit.INCH);}




}
