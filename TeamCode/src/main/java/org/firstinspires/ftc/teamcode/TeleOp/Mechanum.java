package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;

// cd /Users/linva/AppData/Local/Android/Sdk/platform-tools
// ./adb connect 192.168.43.1:5555

@TeleOp(name="Mechanum", group= "org/firstinspires/ftc/teamcode/TeleOp")

/* This program is the robot's main TeleOp program which gets run
 * consistently every TeleOperated period. This allows for driver control
 * of the drivetrain, and operator control of all other subsystems on the robot.*/

public class Mechanum extends OpMode {
    // Declare the hardwareMap for the robot
    private GearHoundsHardware robot = new GearHoundsHardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private double shift = 1.0;
    private double P1xTime = -10;
    private double P1aTime = -10;
    private double P1lpadTime = -10;
    private double P1rpadTime = -10;




    boolean Moving = false;
    boolean Chaining = false;
    boolean Handing = false;
    boolean ClawOpen = false;
    boolean ClawSideways = false;

    @Override
    public void init() {


        // Intializes the hardwareMap found in "GearHoundsHardware" class
        robot.init(hardwareMap);

    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Hello Linval!");
        telemetry.update();
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();


        //This is the code that changes the color on the Dualshock 4
        gamepad1.setLedColor(17, 0, 17, 1000000000);
        gamepad2.setLedColor(0, 10, 10, 1000000000);
        shift = 1.0;
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
//This is the code that allows the driver to change speed based on controller input.
    {
        if (gamepad1.right_bumper) {
            shift = 1.0;
        }

        if (gamepad1.left_bumper) {
            shift = 0.3;
        }

        if (gamepad1.b) {
            shift = 0.25;
        }

        if (gamepad1.a) {
            shift = 0.5;
        }
        if (gamepad1.x) {
            shift = 0.75;
        }

        if (gamepad1.y) {
            shift = 1.0;
        }


//This code allows you to move the linear actuator in and out with limits and changeable speed
        if (gamepad1.dpad_down && robot.linear.getCurrentPosition() < -200) {  // 5030 is upper limit on Linear Actuator for future me
            robot.linear.setVelocity(1000000);
        } else if (gamepad1.dpad_up && robot.linear.getCurrentPosition() > -4150) { // 740 is lower limit on Linear Actuator for future me
            robot.linear.setVelocity(-1000000);
        } else {
            robot.linear.setPower(0);
        }


        if (gamepad1.right_trigger > 0.8) {
            robot.claw.setPosition(0.67);
            ClawOpen = false;
        }
        if (gamepad1.left_trigger > 0.8) {
            robot.claw.setPosition(0.624);
            ClawOpen = true;
        }

        if (gamepad1.dpad_left) {
            P1lpadTime = runtime.seconds();
        }
            if (((runtime.seconds() - P1lpadTime) < 0.03) && ClawOpen == true) {
                robot.claw.setPosition(0.67);
            } else if ((runtime.seconds() - P1lpadTime) < 0.04) {
                robot.rotate.setPosition(0.8);
                ClawOpen = false;
                ClawSideways = true;
            }

        if (gamepad1.dpad_right) {
            P1rpadTime = runtime.seconds();
        }
            if (((runtime.seconds() - P1rpadTime) < 0.05) && ClawOpen == true) {
                robot.claw.setPosition(0.67);
            } else if ((runtime.seconds() - P1rpadTime) < 0.06) {
                robot.rotate.setPosition(0.485);
                ClawOpen = false;
                ClawSideways = false;
            }

        if (gamepad1.x) {
            P1xTime = runtime.seconds();
        }

            if (((runtime.seconds() - P1xTime) < 1) && ClawSideways == false) {
                robot.rotate.setPosition(0.485);
                robot.wrist.setPosition(0.845);
            } else if (((runtime.seconds() - P1xTime) < 1.05) && ClawSideways == false) {
                robot.claw.setPosition(0.659);
                ClawSideways = false;
            }

            if (((runtime.seconds() - P1xTime) < 0.2) && ClawSideways == true) {
                robot.claw.setPosition(0.67);
            } else if (((runtime.seconds() - P1xTime) < 1.1) && ClawSideways == true) {
                robot.rotate.setPosition(0.485);
                robot.wrist.setPosition(0.845);
                robot.claw.setPosition(0.659);
                ClawSideways = false;
            }


        if (gamepad1.a) {
            P1aTime = runtime.seconds();
        }

            if ((runtime.seconds() - P1aTime) < 1) {
                robot.rotate.setPosition(0.485);
                robot.wrist.setPosition(0);
                robot.claw.setPosition(0.67);
            } else if ((runtime.seconds() - P1aTime) < 1.05) {
                robot.claw.setPosition(0.623);
                ClawOpen = true;
            }

//This code allows you to move the lift up and down with limits and changeable speed
            if (-gamepad2.left_stick_y > 0 && robot.leftLift.getCurrentPosition() > -3500) {  // -5955 is upper limit on lift for future me
                robot.leftLift.setVelocity(gamepad2.left_stick_y * 2000);
                robot.rightLift.setVelocity(gamepad2.left_stick_y * 2000);
                robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (-gamepad2.left_stick_y < 0 && robot.leftLift.getCurrentPosition() < 1) {
                robot.leftLift.setVelocity(gamepad2.left_stick_y * 2000);
                robot.rightLift.setVelocity(gamepad2.left_stick_y * 2000);
                robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else robot.leftLift.setPower(0);
                   robot.rightLift.setPower(0);


            if (gamepad2.dpad_down){
                robot.arm.setVelocity(-500);
            } else if (gamepad2.dpad_up){
                robot.arm.setVelocity(500);
            } else robot.arm.setVelocity(0);

            if (gamepad2.ps){
                robot.UpClawL.setPosition(0);
                robot.UpClawR.setPosition(0);
            }

            if (gamepad2.left_bumper){
                robot.UpClawL.setPosition(0);
                robot.UpClawR.setPosition(0.65);
            }

            if (gamepad2.right_bumper){
                robot.UpClawL.setPosition(0.28);
                robot.UpClawR.setPosition(0.3);
            }

            if (gamepad2.b){
                robot.leftLift.setTargetPosition(-3500);
                robot.rightLift.setTargetPosition(-3500);
                robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftLift.setPower(0.2);
                robot.rightLift.setPower(0.2);
            }
//
//        if (gamepad2.right_bumper)
//            robot.claw.setPosition(.5);
//
//        if (gamepad2.left_bumper)
//            robot.sticks.setPosition(.45);
//
//        if (gamepad2.left_trigger > 0.1) {
//            Chaining = false;
//            robot.chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.chain.setPower(-gamepad2.left_trigger);
//        } else if (gamepad2.right_trigger > 0.1) {
//            Chaining = false;
//            robot.chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.chain.setPower(gamepad2.right_trigger);
//        } else {
//            if (Chaining == false) {
//                robot.chain.setPower(0);
//            }
//        }
//
//        if (-gamepad2.right_stick_y > 0) {
//            Moving = false;
//            robot.hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.hanger.setPower(-gamepad2.right_stick_y*4);
//        } else if (-gamepad2.right_stick_y < 0 && robot.hanger.getCurrentPosition() > 40) {
//            Moving = false;
//            robot.hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.hanger.setPower(-gamepad2.right_stick_y*4);
//        } else {
//            if (Moving == false) {
//                robot.hanger.setPower(0);
//            }
//        }
//
////        if (gamepad2.left_trigger > 0.5) {
////            robot.chain.setPower(-0.5);
////        } else if (gamepad2.right_trigger > 0.5) {
////            robot.chain.setPower(0.3);
////        } else {
////            robot.chain.setPower(0);
////        }
//
//        if (gamepad2.left_stick_y < 0) {
//            Moving = false;
//            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.lift.setPower(gamepad2.left_stick_y);
//        } else if (gamepad2.left_stick_y > 0 && robot.lift.getCurrentPosition() < 40) {
//            Moving = false;
//            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.lift.setPower(gamepad2.left_stick_y*0.85);
//        } else {
//            if (Moving == false) {
//                robot.lift.setPower(0);
//            }
//        }
//
//        if (gamepad2.left_stick_button) {
//            Moving = true;
//            gamepad2.rumble(100);
//        }
//
//        if (gamepad2.dpad_up) {
//            robot.launcher.setPosition(1);
//        }
//
//        if (gamepad2.dpad_down)
//            robot.launcher.setPosition(-1);
////        if () {
////            robot.chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.chain.setTargetPosition(29);
////            robot.chain.setPower(0.5);
////
////    }
//        /*if (gamepad1.right_bumper)
//            robot.intake.setPower(1);
//        else robot.intake.setPower(0);
//
//
//        if (gamepad1.left_bumper)
//            robot.intake.setPower(-1);
//        else robot.intake.setPower(0);
//*/

            telemetry.addData("", "Left Front %d   Right Front %d", robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());

            telemetry.addData("", "Left Back %d   Right Back %d", robot.leftBack.getCurrentPosition(), robot.rightBack.getCurrentPosition());

            telemetry.addData("", "Angle  %f", robot.getAngle());

            telemetry.addData("", "Linear %d", robot.linear.getCurrentPosition());


//        telemetry.addData("", "chain %d", robot.chain.getCurrentPosition());
//
            telemetry.addData("", "Left Lift %d  Right Lift %d", robot.leftLift.getCurrentPosition(), robot.rightLift.getCurrentPosition());
//


//
//            telemetry.addData("", "wrist %f", robot.wrist.getPosition());
//
//            telemetry.addData("", "elbow %f", robot.elbow.getPosition());
//
//            telemetry.addData("", "Left Claw %f   Right Claw %f", robot.leftClaw.getPosition(), robot.rightClaw.getPosition());

            telemetry.addData("", "Driver Speed %f", shift);

//            telemetry.addData("", "Manual Mode", manual);

//            telemetry.addData("", "Left Trigger %d", gamepad2.left_trigger);


            //telemetry.addData("", "claw %d", robot.claw.getCurrentPosition());

            telemetry.update();

            //code taken from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
            double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            if (gamepad1.options) {
                robot.imu.resetYaw();
            }


//        public void drive_update() {
//            /**gets the angle from the imu**/
//            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//            double angle = angles.firstAngle;
//            /**gets squared values from the driver's stick input**/
//            double r = Math.hypot(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
//            /**finds the desired angle that the driver wants to move the robot**/
//            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//            /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
//             * the offset value is set by the the driver if the imu does not reset after auto*/
//            robotAngle = robotAngle - Math.toRadians(angle) + offset;
//            /** sets the turning value */
//            double rightX = gamepad1.right_stick_x;
//            /** calculates the motor powers using trig functions and previously found values */
//            final double v1 = r * Math.cos(robotAngle) + rightX;
//            final double v2 = r * Math.sin(robotAngle) - rightX;
//            final double v3 = r * Math.sin(robotAngle) + rightX;
//            final double v4 = r * Math.cos(robotAngle) - rightX;


            double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
            rotX = rotX * 1.1;
            double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

            double d = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double lf = (rotY + rotX + rx) / d;
            double lb = (rotY - rotX + rx) / d;
            double rf = (rotY - rotX - rx) / d;
            double rb = (rotY + rotX - rx) / d;

            //set rightFront negative so it goes same direction as other heels
            robot.leftFront.setVelocity(2000 * lf * shift);
            robot.leftBack.setVelocity(2000 * lb * shift);
            robot.rightFront.setVelocity(2000 * rf * shift);
            robot.rightBack.setVelocity(2000 * rb * shift);

        }
//
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            robot.leftFront.setPower(frontLeftPower);
//            robot.leftBack.setPower(backLeftPower);
//            robot.rightFront.setPower(frontRightPower);
//            robot.rightBack.setPower(backRightPower);
//        }
//



        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop () {

            // set all the motors to stop
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
//            robot.lift.setPower(0);
            //robot.spinner.setPower(0);
            //robot.claw.setPower(0);
//        robot.claw.setPosition(0);
        }

//    // function used to make sure that the value inputted to the
//    // motors stays between 1 and -1
//    private double clamp(double x, double min, double max) {
//
//        return Math.max(min,Math.min(max,x));
//
//    }


    }


