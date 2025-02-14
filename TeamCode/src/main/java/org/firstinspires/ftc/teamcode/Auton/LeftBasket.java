package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonCommands.Arm;
import org.firstinspires.ftc.teamcode.AutonCommands.MoveForDistance;
import org.firstinspires.ftc.teamcode.AutonCommands.RepositionFrontSensor;
import org.firstinspires.ftc.teamcode.AutonCommands.SetClaw;
import org.firstinspires.ftc.teamcode.AutonCommands.SetClaws;
import org.firstinspires.ftc.teamcode.AutonCommands.SlideToPosition;
import org.firstinspires.ftc.teamcode.AutonCommands.StrafeForDistance;
import org.firstinspires.ftc.teamcode.AutonCommands.TurnByAngle;
import org.firstinspires.ftc.teamcode.AutonCommands.WaitForTime;
import org.firstinspires.ftc.teamcode.Utilities.Command;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "LeftBasket")
public class LeftBasket extends LinearOpMode {
    private GearHoundsHardware robot = new GearHoundsHardware();
    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();
    private boolean done = false;
    private double propPos = 0;

    @Override
    public void runOpMode() {
        double startTime = 0;
        robot.init(hardwareMap);
        robot.imu.resetYaw();
        List<Command> steps = new ArrayList<>();
        int step = 0;
        while (!isStarted()) {
            telemetry.update();
        }

        telemetry.update();
//        robot.leftClaw.setPosition(1);
//
//        robot.rightClaw.setPosition(-0.6);
//
//        robot.wrist.setPosition(0)


        steps.add(new SetClaws(robot, runtime, 1, 0.28, 0.3));
        steps.add(new SetClaw(robot, runtime, 1, 0.624, 3));
        steps.add(new SlideToPosition(robot, runtime, -3500, 1, 8));
        steps.add(new Arm(robot,runtime, 45, 0.5, 4));
        steps.add(new SetClaws(robot,runtime, 1,0,0.65));
//        steps.add(new RepositionFrontSensor(robot, runtime, 5, 2, -0.7, true, 0, 0.06));
//        steps.add(new StrafeForDistance(robot,24 , 8, 9, runtime, 5, -0.5, 1));
//        steps.add(new StrafeForDistance(robot, 15, 4, 1, runtime, 5, 0.5, 1));
//        steps.add(new RepositionFrontSensor(robot, runtime, 60, 2, -0.7, true, 0, 0.06));

//        steps.add(new StrafeForDistance(robot,20 , 1, 1, runtime, 5, 0.5, 1));
//        steps.add(new StrafeForDistance(robot, 20, 1, 1, runtime, 5, -0.5, 1));
//        steps.add(new MoveForDistance(robot, 43, 3, 4, runtime, 5, -0.5, 1));
//        steps.add(new StrafeForDistance(robot, 24, 3, 6, runtime, 5, 0.5, 1));
//        steps.add(new MoveForDistance(robot, 37, 5, 4, runtime, 5, 1, 0.5));
//        steps.add(new MoveForDistance(robot, 37, 1, 3, runtime, 5, -1, 0.5));
//        steps.add(new StrafeForDistance(robot, 4, 1,2 , runtime, 5, 0.5, 1));
//        steps.add(new MoveForDistance(robot, 35, 3, 4, runtime, 5, 0.5, 1));
//        steps.add(new StrafeForDistance(robot, 14, 1, 3, runtime, 5, -0.5, 1));


        //steps.add(new MoveForDistance(robot, 20, 5, 5, runtime, 5, 0.5, 1));

//        if (propPos == 1) {
//            steps.add(new MoveForDistance(robot, 3, 1, 1, runtime, 5, -0.5, 1));
//            steps.add(new StrafeForDistance(robot, 9, 3, 1, runtime, 3, 0.25, 1 ));
//            steps.add(new MoveForDistance(robot, 16, 6, 3, runtime, 5, 0.75, 0.75));
//            steps.add(new WaitForTime(robot, runtime, 1));
//            steps.add(new StrafeForDistance(robot,25  , 3, 1, runtime, 3, 0.25, 1 ));
//            robot.claw.setPosition(9);
//        } else if (propPos == 2) {
//            steps.add(new MoveForDistance(robot, 9, 1, 1, runtime, 5, -0.25, 1));
//            steps.add(new MoveForDistance(robot, 5, 4, 0.5, runtime, 5, 0.75, 0.75));
//            steps.add(new StrafeForDistance(robot,25 , 3, 1, runtime, 3, 0.75, 1 ));
//            steps.add(new WaitForTime(robot, runtime, 1));
//            robot.claw.setPosition(9);
//        } else if (propPos == 3) {
//            steps.add(new TurnByAngle(robot, runtime, 45, 0.5, 5));
//            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, -0.5, 1));
//            steps.add(new WaitForTime(robot, runtime, 1));
//            robot.claw.setPosition(9);
//            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, -0.5, 1));
//            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, 0.5, 1));
//            steps.add(new MoveForDistance(robot, 28,25 , 3, runtime, 5, 0.5, 1));
//            steps.add(new StrafeForDistance(robot,10 , 3, 1, runtime, 3, 0.75, 1 ));
//        }

//        steps.add(new MoveForDistance(robot, 300, 100, 100, runtime, 5, -0.5, 1));
//        steps.add(new SlideToPosition(robot, runtime, 500, -0.35, 5));

        // This is where we build the autonomous routine
        Command currentStep = steps.get(step);
        while (opModeIsActive() && !done) {
            // Execute current command
            if (currentStep.getState() == Command.STARTING) {
                telemetry.addData("Step:", "Starting");
                currentStep.init();
            } else if (currentStep.getState() == Command.RUNNING) {
                telemetry.addData("Step:", "Running");
                currentStep.run();
            } else if (currentStep.getState() == Command.ENDING) {
                telemetry.addData("Step:", "Ending");
                currentStep.end();
            } else if (currentStep.getState() == Command.DONE) {
                telemetry.addData("Step:", "Done");
                step++;
                if (step >= steps.size()) {
                    done = true;
                } else {
                    currentStep = steps.get(step);
                }
            }
            telemetry.update();
        }

    }}
