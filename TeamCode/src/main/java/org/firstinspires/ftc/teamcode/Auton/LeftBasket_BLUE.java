package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonCommands.Arm;
import org.firstinspires.ftc.teamcode.AutonCommands.MoveForDistance;
import org.firstinspires.ftc.teamcode.AutonCommands.RepositionFrontSensor;
import org.firstinspires.ftc.teamcode.AutonCommands.Rotate;
import org.firstinspires.ftc.teamcode.AutonCommands.SetClaw;
import org.firstinspires.ftc.teamcode.AutonCommands.SetClaws;
import org.firstinspires.ftc.teamcode.AutonCommands.SetWrist;
import org.firstinspires.ftc.teamcode.AutonCommands.SlideToPosition;
import org.firstinspires.ftc.teamcode.AutonCommands.StrafeForDistance;
import org.firstinspires.ftc.teamcode.AutonCommands.TurnToHeading;
import org.firstinspires.ftc.teamcode.Utilities.Command;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "LeftBasket")
public class LeftBasket_BLUE extends LinearOpMode {
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

        steps.add(new StrafeForDistance(robot,6 , 1, 3, runtime, 3, 0.5, 1));
        steps.add(new TurnToHeading(robot, runtime, 39, -0.4, 3));
        steps.add(new MoveForDistance(robot,3 , 1, 1, runtime, 3, 1, 0.5));
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        steps.add(new SetClaws(robot, runtime, 1, 0.28, 0.3));
        steps.add(new SlideToPosition(robot, runtime, -3500, 1, 10));
        steps.add(new Arm(robot,runtime, 160, 0.5, 3));
        steps.add(new SetClaws(robot,runtime, 1,0,0.65));
        steps.add(new Arm(robot,runtime, 0, 1, 1));
        steps.add(new SlideToPosition(robot, runtime, 0, 1, 8));
        steps.add(new TurnToHeading(robot, runtime, 90, -0.5, 3));
        steps.add(new Rotate(robot,runtime, 1, 0.56));
        steps.add(new SetWrist(robot,runtime, 1, 0.08));
        steps.add(new StrafeForDistance(robot,1 , 0.5, 0.5, runtime, 2, -0.5, 1));
        steps.add(new SetClaw(robot, runtime, 1, 0.624, 3));
        steps.add(new MoveForDistance(robot,10 , 1, 3, runtime, 3, -0.3, 0.5));
        steps.add(new SetClaw(robot, runtime, 1, 0.67, 0));
        steps.add(new Rotate(robot,runtime, 1, 0.89));
        steps.add(new SetWrist(robot,runtime, 1, 0.88));
        steps.add(new SetClaws(robot, runtime, 1, 0.28, 0.3));
        steps.add(new SetClaws(robot, runtime, 1, 0.28, 0.3));




//            steps.add(new MoveForDistance(robot, 16, 6, 3, runtime, 5, 0.75, 0.75));
//            steps.add(new WaitForTime(robot, runtime, 1));

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
