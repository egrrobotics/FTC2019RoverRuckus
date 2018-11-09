package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.PID;

/**
 * Created by Howard Paul on 11/8/2018.
 */

public class DriveForwardHeadingandDistanceSensor extends BasicCommand {
    double targetPosition;
    double endDistance;
    double driveSpeed;
    double heading;
    double leftSpeed;
    double rightSpeed;
    double correction;
    double distanceCorrection;
    PID headingPID;
    PID distancePID;
    public static final int FRONTGREATERTHAN = 0;
    public static final int BACKGREATERTHAN = 1;
    public static final int FRONTLESSTHAN = 2;
    public static final int BACKLESSTHAN = 3;
    int test;
    long endTime;
    double targetHeading;
    boolean coast = false;
    public DriveForwardHeadingandDistanceSensor(double targetPosition, int test, double spd, double targetHeading){
        headingPID = new PID(0.05,0,0);
        headingPID.setTarget(targetHeading);
        distancePID = new PID(0.2,0,0);
        distancePID.setTarget(targetPosition);
        this.targetPosition = targetPosition;
        this.test = test;
        driveSpeed = spd;
        this.targetHeading = targetHeading;
    }

    public void init(){
        endTime = System.currentTimeMillis() + 500000;
    }

    public void execute(){

        heading = Math.toDegrees(io.heading);
        correction = headingPID.getCorrection(heading);

        switch(test) {
            case FRONTGREATERTHAN:
            case FRONTLESSTHAN:
                distanceCorrection = distancePID.getCorrection(io.frontDistance.getDistance(DistanceUnit.INCH));
                break;
            case BACKGREATERTHAN:
            case BACKLESSTHAN:
            default:
                distanceCorrection = distancePID.getCorrection(io.backDistance.getDistance(DistanceUnit.INCH));
                break;
        }
        distanceCorrection = Range.clip(Math.abs(distanceCorrection),0,1);
        correction = Range.clip(correction,-1,1);
        leftSpeed = (driveSpeed * distanceCorrection) + correction;
        rightSpeed = (driveSpeed * distanceCorrection) - correction;
        if (driveSpeed > 0) {
            leftSpeed = Range.clip(leftSpeed, 0, 1);
            rightSpeed = Range.clip(rightSpeed, 0, 1);
        } else {
            leftSpeed = Range.clip(leftSpeed, -1, 0);
            rightSpeed = Range.clip(rightSpeed, -1, 0);
        }

        io.setDrivePower(leftSpeed,rightSpeed);
        telemetry.addData("Front Distance: ",io.frontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Distance: ",io.backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Left Speed: ", leftSpeed);
        telemetry.addData("Right Speed: ", rightSpeed);
        telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());
        telemetry.addData("Mode:", "Drive Forward Heading and Distance Sensor");
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= endTime) return true;
        telemetry.addData("Front Distance: ",io.frontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Distance: ",io.backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Left Speed: ", leftSpeed);
        telemetry.addData("Right Speed: ", rightSpeed);
        telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());
        telemetry.addData("Mode:", "Drive Forward Heading and Distance Sensor");

        switch(test) {
            case FRONTGREATERTHAN:
                return io.frontDistance.getDistance(DistanceUnit.INCH) > targetPosition;
            case FRONTLESSTHAN:
                return io.frontDistance.getDistance(DistanceUnit.INCH) < targetPosition;
            case BACKGREATERTHAN:
                return io.backDistance.getDistance(DistanceUnit.INCH) > targetPosition;
            case BACKLESSTHAN:
            default:
                return io.backDistance.getDistance(DistanceUnit.INCH) < targetPosition;
        }
    }

    public void stop(){
        if (!coast) io.setDrivePower(0.0,0.0);
    }

}
