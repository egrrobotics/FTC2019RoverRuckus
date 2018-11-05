package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.utilities.*;

import java.util.ArrayList;
import java.util.Iterator;


/**
 * Created by HPaul on 10/22/2017.
 */

public class RaiseDOM extends BasicCommand {
    long timeOut;
    ArrayList<BasicCommand> commandsInitDOM1;
    ArrayList<BasicCommand> commandsInitDOM2;
    BasicCommand currentCommandInitDOM1;
    BasicCommand currentCommandInitDOM2;
    Iterator<BasicCommand> iteratorInitDOM1;
    Iterator<BasicCommand> iteratorInitDOM2;

    public RaiseDOM(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 10000;
        commandsInitDOM1 = new ArrayList<BasicCommand>();
        commandsInitDOM2 = new ArrayList<BasicCommand>();

        commandsInitDOM1.add(new DOM1Movement(30, DOM1Movement.INCREASINGDIRECTION, .5));
        commandsInitDOM2.add(new DOM2Movement(30, DOM2Movement.INCREASINGDIRECTION, .5));

        iteratorInitDOM1 = commandsInitDOM1.iterator();
        iteratorInitDOM2 = commandsInitDOM2.iterator();

        currentCommandInitDOM1 = iteratorInitDOM1.next();
        currentCommandInitDOM2 = iteratorInitDOM2.next();
    }

    public void execute(){
        telemetry.addData("Mode:", "Raise DOM");
        currentCommandInitDOM1.execute();
        currentCommandInitDOM2.execute();
    }

    public boolean isFinished(){
        return System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.dom1Motor.setPower(0);
        io.dom2Motor.setPower(0);
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}
