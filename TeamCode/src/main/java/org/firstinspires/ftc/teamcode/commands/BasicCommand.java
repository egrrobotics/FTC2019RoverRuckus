package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

/**
 * Created by David Austin on 10/27/2016.
 */

public class BasicCommand {
    static IO_4WD_Test io;
    static Telemetry telemetry;
    public static void setIO(IO_4WD_Test i) {
        io = i;
    }
    public static IO_4WD_Test getIO() {
        return io ;
    }
    public static void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    public void init(){
    }

    public void execute(){
    }

    public void stop(){
    }

    public boolean isFinished(){
        return false;
    }
}
