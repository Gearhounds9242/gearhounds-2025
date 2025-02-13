package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Command;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.Hardware;

public class RepositionFrontSensor extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private GearHoundsHardware robot;
    private double powerLevel;

    private int leftFrontTicks;
    private int rightFrontTicks;
    private int leftFrontStartTicks;
    private int rightFrontStartTicks;
    private int leftEndTicks;
    private int rightEndTicks;
    private double distance;
    private double measure;
    private double startMeasure;

    private double heading;
    private double gain;
    private boolean correction;

    public RepositionFrontSensor(Hardware robot, ElapsedTime timer, double distance, double timeOut, double powerLevel, boolean correction, double heading, double gain) {
        super(robot);
        this.timer = timer;
        this.distance = distance;
        if (timeOut<0){
            timeOut=30;
        }
        this.timeOut = timeOut*1000;
        this.powerLevel = powerLevel;
        this.robot = (GearHoundsHardware) getRobot();
        leftFrontTicks=0;
        rightFrontTicks=0;
        setState(STARTING);
        this.correction = correction;
        this.heading = heading;
        this.gain = gain;
    }

    public void init() {
        if (getState() == STARTING) {
            setState(RUNNING);
            startTime = timer.milliseconds();
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontStartTicks=robot.leftFront.getCurrentPosition();
            rightFrontStartTicks=robot.rightFront.getCurrentPosition();
//            leftEndTicks=leftFrontTicks + (int)(-(inches*GearHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
//            rightEndTicks=rightFrontTicks + (int)(-(inches*GearHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
//            startMeasure = robot.range.getDistance(DistanceUnit.INCH);
            if (!correction) {
                heading = robot.getAngle();
            }
        }
    }


    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            if (elapsedTime > timeOut) {
                setState(ENDING);
            } else {
                double currentAngle = robot.getAngle();
                double angleError = heading - currentAngle;
                double correctionFactor = robot.clamp(Math.abs(gain * angleError * powerLevel), 0, powerLevel);
                measure = robot.getDistance(robot.ranger);
                double powerFactor = Math.min(Math.abs(startMeasure - distance - (startMeasure - measure))/20.0,1.0);
                if (angleError > 0) {
                    if (measure > distance + 0.3) {
                        robot.leftFront.setPower(powerLevel * powerFactor - correctionFactor);
                        robot.leftBack.setPower(powerLevel * powerFactor - correctionFactor);
                        robot.rightFront.setPower(powerLevel * powerFactor);
                        robot.rightBack.setPower(powerLevel * powerFactor);
                    } else if (measure < distance - 0.3) {
                        robot.leftFront.setPower(-powerLevel * powerFactor - correctionFactor);
                        robot.leftBack.setPower(-powerLevel * powerFactor - correctionFactor);
                        robot.rightFront.setPower(-powerLevel * powerFactor);
                        robot.rightBack.setPower(-powerLevel * powerFactor);
                    } else {
                        setState(ENDING);
                    }
                } else {
                    if (measure > distance + 0.3) {
                        robot.leftFront.setPower(powerLevel * powerFactor);
                        robot.leftBack.setPower(powerLevel * powerFactor);
                        robot.rightFront.setPower(powerLevel * powerFactor - correctionFactor);
                        robot.rightBack.setPower(powerLevel * powerFactor - correctionFactor);
                    } else if (measure < distance - 0.3) {
                        robot.leftFront.setPower(-powerLevel * powerFactor);
                        robot.leftBack.setPower(-powerLevel * powerFactor);
                        robot.rightFront.setPower(-powerLevel * powerFactor - correctionFactor);
                        robot.rightBack.setPower(-powerLevel * powerFactor - correctionFactor);
                    } else {
                        setState(ENDING);
                    }
                }
            }
        }
    }

    public void end() {
        if (getState() == ENDING) {
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);
        }
        setState(DONE);
    }

}