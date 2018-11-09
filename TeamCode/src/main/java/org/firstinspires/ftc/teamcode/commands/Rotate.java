package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.utilities.HSPID;

/**
 * Created by David Austin on 10/27/2016.
 */

public class Rotate extends BasicCommand {
    double heading,leftSpd,rightSpd;
    PID headingPID;
    long timeOut;

    public Rotate (double heading,double leftSpd,double rightSpd){
        this.heading = heading;
        this.leftSpd = leftSpd;
        this.rightSpd = rightSpd;
        headingPID = new PID(0.05,0,0); //was 0.05
        headingPID.setTarget(heading);
    }

    public void init() {
        timeOut = System.currentTimeMillis() + 5000;
    }

    public void execute(){
        //double correction = headingPID.getCorrection(io.getHeading());
        double correction = headingPID.getCorrection(Math.toDegrees(io.heading));
        correction = Range.clip(correction,-1,1);
        //correction = Range.clip(correction,0,1);
        io.setDrivePower(correction*leftSpd,-correction*rightSpd);
        telemetry.addData("Target Heading:", heading);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Correction: ", correction);
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        telemetry.addData("Mode:", "Rotate");
    }

    public boolean isFinished(){
        telemetry.addData("Target Heading:", heading);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("x: ",io.getX());
        telemetry.addData("y: ",io.getY());
        telemetry.addData("Correction: ", headingPID.getCorrection(Math.toDegrees(io.heading)));
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        //return Math.abs(io.getHeading() - heading) <=2 || System.currentTimeMillis() >= timeOut;
        return Math.abs(Math.toDegrees(io.heading) - heading) <=2 || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0);
    }

}
