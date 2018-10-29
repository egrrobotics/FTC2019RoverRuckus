package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.utilities.HSPID;
import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

/**
 * Created by David Austin on 10/27/2016.
 */

public class DriveForward extends BasicCommand {
    double targetPosition;
    double endDistance;
    double driveSpeed;
    PID distancePID;
    PID headingPID;
    public static final int YGREATERTHAN = 0;
    public static final int XGREATERTHAN = 1;
    public static final int YLESSTHAN = 2;
    public static final int XLESSTHAN = 3;
    int test;
    long endTime;
    double targetHeading;
    boolean coast = false;
    boolean proximitycorrection = false;
    boolean proximitybutton = false;
    boolean usebutton = false;
    public DriveForward(double targetPosition, int test, double spd, double targetHeading){
        headingPID = new PID(0.05,0,0);
        //headingPID = new PID(0.02, 0.02, 0);
        //headingPID = new PID(0.05, 0, 0);
        headingPID.setTarget(targetHeading);
        distancePID = new PID(.2,0,0);
        //distancePID = new PID(.2,0,0);
        distancePID.setTarget(targetPosition);
        this.targetPosition = targetPosition;
        this.test = test;
        driveSpeed = spd;
        this.targetHeading = targetHeading;
    }
    public DriveForward(double targetPosition, int test, double spd, double targetHeading, boolean coast){
        this(targetPosition,test,spd,targetHeading);
        this.coast=coast;
    }
    public DriveForward(double targetPosition, int test, double spd, double targetHeading, boolean coast, boolean proximitycorrection){
        this(targetPosition,test,spd,targetHeading);
        this.coast=coast;
        this.proximitycorrection = proximitycorrection;
    }
    public DriveForward(double targetPosition, int test, double spd, double targetHeading, boolean coast, boolean proximitycorrection, boolean usebutton){
        this(targetPosition,test,spd,targetHeading);
        this.coast=coast;
        this.proximitycorrection = proximitycorrection;
        this.usebutton = usebutton;
    }
    public DriveForward(double dist) {
        this(dist, YGREATERTHAN, 0.5, 0.0);
    }

    public void init(){
        endTime = System.currentTimeMillis() + 10000;

        if (proximitycorrection){
            headingPID.setTarget(targetHeading + io.getProximityCorrection());
        }
        if (usebutton){
            this.proximitybutton = io.proximityArmButtonPushed;
        }
    }

    public void execute(){
        //double heading = io.getHeading();
        double heading = Math.toDegrees(io.heading);
        double correction = headingPID.getCorrection(heading);
        double distanceCorrection;
        switch(test) {
            case XGREATERTHAN:
            case XLESSTHAN:
                distanceCorrection = distancePID.getCorrection(io.getX());
                break;
            case YGREATERTHAN:
            case YLESSTHAN:
            default:
                distanceCorrection = distancePID.getCorrection(io.getY());
                break;
        }
        distanceCorrection = Range.clip(Math.abs(distanceCorrection),0,1);
        correction = Range.clip(correction,-1,1);
        double leftSpeed = (driveSpeed * distanceCorrection) + correction;
        double rightSpeed = (driveSpeed * distanceCorrection) - correction;
        if (driveSpeed > 0) {
            leftSpeed = Range.clip(leftSpeed, 0, 1);
            rightSpeed = Range.clip(rightSpeed, 0, 1);
        } else {
            leftSpeed = Range.clip(leftSpeed, -1, 0);
            rightSpeed = Range.clip(rightSpeed, -1, 0);
        }

        io.setDrivePower(leftSpeed,rightSpeed);
        telemetry.addData("x: ",io.getX());
        telemetry.addData("y: ",io.getY());
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Left Speed: ", leftSpeed);
        telemetry.addData("Right Speed: ", rightSpeed);
        telemetry.addData("Jewel Color is Unknown, Red, Blue: ", io.getJewelColor());
        telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());
        telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        telemetry.addData("VuMark from IdentifyVuMark from IO", "%s visible", io.vuMark);
        telemetry.addData("Mode:", "Drive Forward");
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= endTime) return true;
        if ((io.touchProximity.getState() == false) && (usebutton == false)){
            io.proximityArmButtonPushed = true;
            return true;
        }
        if ((usebutton == true) && (proximitybutton == false)) return true;
        //telemetry.addData("x: ",io.getX());
        //telemetry.addData("y: ",io.getY());
        //telemetry.addData("Target Heading:", targetHeading);
        //telemetry.addData("Heading:", io.getHeading());
        //telemetry.addData("Heading Correction: ", headingPID.getCorrection(io.getHeading()));
        //telemetry.addData("Distance Correction: ", distancePID.getCorrection(io.getY()));
        //telemetry.addData("Drive Speed: ", driveSpeed);
        //telemetry.addData("Left Speed: ", (driveSpeed * distancePID.getCorrection(io.getY())) - headingPID.getCorrection(io.getHeading()));
        //telemetry.addData("Right Speed: ", (driveSpeed * distancePID.getCorrection(io.getY())) + headingPID.getCorrection(io.getHeading()));
        switch(test) {
            case XGREATERTHAN:
                return io.getX() > targetPosition;
            case XLESSTHAN:
                return io.getX() < targetPosition;
            case YGREATERTHAN:
                return io.getY() > targetPosition;
            case YLESSTHAN:
            default:
                return io.getY() < targetPosition;
        }
    }

    public void stop(){
        if (!coast) io.setDrivePower(0.0,0.0);
    }

}
