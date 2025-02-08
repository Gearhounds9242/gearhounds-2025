package org.firstinspires.ftc.teamcode.AutonCommands;

import org.firstinspires.ftc.teamcode.Utilities.Command;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;



//package org.firstinspires.ftc.teamcode.AutonCommands;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utilities.Command;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.Hardware;

public class SetClaws extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double runTime;
    private GearHoundsHardware robot;
    private double servoPosition;
    private double servoPosition1;
    private boolean useInit;
    private boolean useEnd;

    public SetClaws(Hardware robot, ElapsedTime timer, double runTime, double servoPosition, double servoPosition1) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.servoPosition = servoPosition;
        this.servoPosition1 = servoPosition1;
        this.robot = (GearHoundsHardware) getRobot();
        this.useInit = true;
        this.useEnd = true;
        setState(STARTING);
    }

    public SetClaws(Hardware robot, ElapsedTime timer, double runTime, double servoPosition, boolean useInit, boolean useEnd) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.servoPosition = servoPosition;
        this.robot = (GearHoundsHardware) getRobot();
        this.useInit = useInit;
        this.useEnd = useEnd;
        setState(STARTING);
    }

    public void init() {
        startTime = timer.milliseconds();
        setState(RUNNING);
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            if (elapsedTime < runTime) {
                //robot.claw.setPosition(servoPosition);
                robot.UpClawL.setPosition(servoPosition);
                robot.UpClawR.setPosition(servoPosition1);
            } else {
                setState(ENDING);
            }
        }
    }

    public void end() {
        setState(DONE);
    }

}