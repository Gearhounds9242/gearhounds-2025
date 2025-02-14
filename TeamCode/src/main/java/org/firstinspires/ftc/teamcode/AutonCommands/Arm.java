package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Command;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.Hardware;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;

public class Arm extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private GearHoundsHardware robot;
    private double powerLevel;
    private int position;
    private boolean useInit;
    private boolean useEnd;

    public Arm(Hardware robot, ElapsedTime timer, int position, double powerLevel, double timeOut) {
        super(robot);
        this.timer = timer;
        this.timeOut = timeOut*1000;
        this.position = position;
        this.powerLevel = powerLevel;
        this.robot = (GearHoundsHardware) getRobot();
        this.useInit = true;
        this.useEnd = true;
        setState(STARTING);
    }

    public Arm(Hardware robot, ElapsedTime timer, int position, double powerLevel, double timeOut, boolean useInit, boolean useEnd) {
        super(robot);
        this.timer = timer;
        this.timeOut = timeOut*1000;
        this.position = position;
        this.powerLevel = powerLevel;
        this.robot = (GearHoundsHardware) getRobot();
        this.useInit = useInit;
        this.useEnd = useEnd;
        setState(STARTING);
    }

    public void init() {
        if (getState() == STARTING && useInit) {
            startTime = timer.milliseconds();
        }
        setState(RUNNING);
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            if (elapsedTime < timeOut && Math.abs(robot.arm.getCurrentPosition()-position) > -5000) {
                robot.arm.setTargetPosition(position);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(powerLevel);
            } else {
                setState(ENDING);
            }


        }
    }

    public void end() {
        if (getState() == ENDING && useEnd) {
            robot.arm.setPower(0);
        }
        setState(DONE);
    }

}
