package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.robotcontroller.internal.HSCamera;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.utilities.*;

import java.util.ArrayList;
import java.util.Iterator;

import static org.firstinspires.ftc.teamcode.utilities.Sleep.sleep;


/**
 * Created by David Austin on 10/27/2016.
 */

public abstract class FirstAuton extends OpMode {
    IO_RoverRuckus_Test io;
    static final int INIT = 0;
    static final int EXECUTE = 1;
    static final int STOP = 2;
    static final int FINISHED = 3;
    int state;
    int initState;
    ArrayList<BasicCommand> commands;
    ArrayList<BasicCommand> commandsInit;
    ArrayList<BasicCommand> commandsInitDOM1;
    ArrayList<BasicCommand> commandsInitDOM2;
    BasicCommand currentCommand;
    BasicCommand currentCommandInit;
    BasicCommand currentCommandInitDOM1;
    BasicCommand currentCommandInitDOM2;
    Iterator<BasicCommand> iterator;
    Iterator<BasicCommand> iteratorInit;
    Iterator<BasicCommand> iteratorInitDOM1;
    Iterator<BasicCommand> iteratorInitDOM2;
    //int allianceColor = IO.RED;

    public FirstAuton() {
        super();
        this.msStuckDetectInit = 20000;
    }

    public void init() {
        //io = new IO_4WD_Test(hardwareMap, telemetry);
        io = new IO_RoverRuckus_Test(hardwareMap, telemetry);
        //io.setAllianceColor(allianceColor);
        BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);
        //io.retractHands();
        //io.openRelicHand();
        //io.jewelArmUp();
        //io.proximityArmUp();
        telemetry.addData("Status", " Arms Initialized");
        io.calibrateGyroandIMU();
        //HSCamera.setHardwareMap(hardwareMap);

/*        //io.setAllianceColor(allianceColor);
        BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);*/

        commands = new ArrayList<BasicCommand>();
        commandsInit = new ArrayList<BasicCommand>();
        commandsInitDOM1 = new ArrayList<BasicCommand>();
        commandsInitDOM2 = new ArrayList<BasicCommand>();
        //addInitCommands();
        addInitDOM1Commands();
        addInitDOM2Commands();
        addCommands();
        addFinalCommands();
        iterator = commands.iterator();
        iteratorInit = commandsInit.iterator();
        iteratorInitDOM1 = commandsInitDOM1.iterator();
        iteratorInitDOM2 = commandsInitDOM2.iterator();
        currentCommand = iterator.next();
        currentCommandInit = iteratorInit.next();
        currentCommandInitDOM1 = iteratorInitDOM1.next();
        currentCommandInitDOM2 = iteratorInitDOM2.next();
        state = INIT;
        initState = INIT;
    }

    /*
 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
 */
    @Override
    public void init_loop() {
        // make sure the gyro is calibrated.
        /*if (io.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro Calibrating. Do Not Move!");
        } else {
            telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        }
*/
        if (!io.imu.isGyroCalibrated()) {
            telemetry.addData(">", "IMU Calibrating. Do Not Move!");
        } else {
            telemetry.addData(">", "IMU Calibrated.  Wait for vuMark Identification.");
        }

        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());

        //if (!currentCommandInit.isFinished()) {
        //if (io.getVuMark() == IO_4WD_Test.UNKNOWN) {
            /*switch (initState) {
                case INIT:
                    currentCommandInit.init();
                    initState = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandInit.isFinished()) {
                        currentCommandInit.stop();
                        if (iteratorInit.hasNext()) {
                            currentCommandInit = iteratorInit.next();
                            initState = INIT;
                        } else initState = EXECUTE; //FINISHED;
                        break;
                    }
                    currentCommandInit.execute();
                    break;
                case STOP:
                    currentCommandInit.stop();
                    if (iteratorInit.hasNext()) {
                        currentCommandInit = iteratorInit.next();
                        initState = INIT;
                    } else initState = FINISHED;
                    break;
                case FINISHED:
                    break;

            }*/
        //}

        /*if (io.getVuMark() == 0) {
            telemetry.addData("vuMark", "Unknown");
            telemetry.addData("vuMark", "not identified, try INIT again");
        } else if (io.getVuMark() == 1) {
            telemetry.addData("vuMark", "Left");
            telemetry.addData("vuMark", "Ready to START");
        } else if (io.getVuMark() == 2) {
            telemetry.addData("vuMark", "Center");
            telemetry.addData("vuMark", "Ready to START");
        } else if (io.getVuMark() == 3) {
            telemetry.addData("vuMark", "Right");
            telemetry.addData("vuMark", "Ready to START");
        }*/
    }

    public void start() {
        //io.setGyroOffset();
        io.setIMUOffset();
        io.resetDriveEncoders();
    }
    public abstract void addCommands();
    public abstract void addFinalCommands();

    /*public void addInitCommands() {
        commandsInit.add(new IdentifyVuMark());
    }*/

    public void addInitDOM1Commands() {
        commandsInitDOM1.add(new DOM1Movement(0, DOM1Movement.INCREASINGDIRECTION, .25));
    }

    public void addInitDOM2Commands() {
        commandsInitDOM2.add(new DOM2Movement(0, DOM2Movement.INCREASINGDIRECTION, .25));
    }

    public void loop() {
        currentCommandInitDOM1.execute();
        currentCommandInitDOM2.execute();
        io.updatePosition();
        switch(state){
            case INIT:
                currentCommand.init();
                state = EXECUTE;
                break;
            case EXECUTE:
                if(currentCommand.isFinished()){
                    currentCommand.stop();
                    if (iterator.hasNext()) {
                        currentCommand = iterator.next();
                        state = INIT;
                    } else state = FINISHED;
                    break;
                }
                currentCommand.execute();
                break;
            case STOP:
                currentCommand.stop();
                if(iterator.hasNext()){
                    currentCommand = iterator.next();
                    state = INIT;
                }else state = FINISHED;
                break;
            case FINISHED:
                break;

        }
    }

    public void stop() {
        io.setDrivePower(0,0);
    }

}
