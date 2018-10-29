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
    IO_4WD_Test io;
    static final int INIT = 0;
    static final int EXECUTE = 1;
    static final int STOP = 2;
    static final int FINISHED = 3;
    int state;
    int initState;
    ArrayList<BasicCommand> commands;
    ArrayList<BasicCommand> commandsInit;
    ArrayList<BasicCommand> commandsInitRPU1;
    ArrayList<BasicCommand> commandsInitRPU2;
    BasicCommand currentCommand;
    BasicCommand currentCommandInit;
    BasicCommand currentCommandInitRPU1;
    BasicCommand currentCommandInitRPU2;
    Iterator<BasicCommand> iterator;
    Iterator<BasicCommand> iteratorInit;
    Iterator<BasicCommand> iteratorInitRPU1;
    Iterator<BasicCommand> iteratorInitRPU2;
    //int allianceColor = IO.RED;

    public FirstAuton() {
        super();
        this.msStuckDetectInit = 20000;
    }

    public void init() {
        io = new IO_4WD_Test(hardwareMap, telemetry);
        //io.setAllianceColor(allianceColor);
        BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);
        io.retractHands();
        io.openRelicHand();
        io.jewelArmUp();
        io.proximityArmUp();
        telemetry.addData("Status", " Arms Initialized");
        io.calibrateGyroandIMU();
        //HSCamera.setHardwareMap(hardwareMap);

/*        //io.setAllianceColor(allianceColor);
        BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);*/

        commands = new ArrayList<BasicCommand>();
        commandsInit = new ArrayList<BasicCommand>();
        commandsInitRPU1 = new ArrayList<BasicCommand>();
        commandsInitRPU2 = new ArrayList<BasicCommand>();
        addInitCommands();
        addInitRPU1Commands();
        addInitRPU2Commands();
        addCommands();
        addFinalCommands();
        iterator = commands.iterator();
        iteratorInit = commandsInit.iterator();
        iteratorInitRPU1 = commandsInitRPU1.iterator();
        iteratorInitRPU2 = commandsInitRPU2.iterator();
        currentCommand = iterator.next();
        currentCommandInit = iteratorInit.next();
        currentCommandInitRPU1 = iteratorInitRPU1.next();
        currentCommandInitRPU2 = iteratorInitRPU2.next();
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

        telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());

        //if (!currentCommandInit.isFinished()) {
        //if (io.getVuMark() == IO_4WD_Test.UNKNOWN) {
            switch (initState) {
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

            }
        //}

        if (io.getVuMark() == 0) {
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
        }
    }

    public void start() {
        //io.setGyroOffset();
        io.setIMUOffset();
        io.resetDriveEncoders();
    }
    public abstract void addCommands();
    public abstract void addFinalCommands();

    public void addInitCommands() {
        commandsInit.add(new IdentifyVuMark());
    }

    public void addInitRPU1Commands() {
        commandsInitRPU1.add(new RPU1Movement(0, RPU1Movement.INCREASINGDIRECTION, .25));
    }

    public void addInitRPU2Commands() {
        commandsInitRPU2.add(new RPU2Movement(0, RPU2Movement.INCREASINGDIRECTION, .25));
    }

    public void loop() {
        currentCommandInitRPU1.execute();
        currentCommandInitRPU2.execute();
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
