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
    private double manual = 1.0;

    boolean Moving = false;
    boolean Chaining = false;

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
        manual = 0.0;
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


        if (gamepad2.options && gamepad2.share) {
            manual = 1;
        }

        if (gamepad2.ps) {
            manual = 0;
        }


        if (gamepad2.options && gamepad2.share && manual == 1) {
            gamepad2.setLedColor(100, 0, 0, 1000000000);
            gamepad2.rumble(100);

        }

        if (gamepad2.ps && manual == 0){
            gamepad2.setLedColor(0, 10, 10, 1000000000);
            gamepad2.rumble(50);
        }



//This code allows you to reset the Linear Actuator by clicking in the right stick (R3) and Options
        if (gamepad2.options && gamepad2.right_stick_button) {
            robot.linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gamepad2.rumbleBlips(2);
            telemetry.addLine("Resetting Linear Encoder");
        }
        telemetry.update();

//This code allows you to reset the Linear Actuator by clicking in the left stick (L3) and Option
        if (gamepad2.options && gamepad2.left_stick_button) {
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gamepad2.rumbleBlips(1);
            telemetry.addLine("Resetting Lift Encoder");
        }
        telemetry.update();



//        if (gamepad2.left_trigger != 0) {
//            robot.claw.setPower(gamepad2.left_trigger*.5);
//        } else if (gamepad2.right_trigger != 0) {
//            robot.claw.setPower(-gamepad2.right_trigger*.5);
//        } else robot.claw.setPower(0);


//This is the code that lets the Operator to open and close the claw, via the left and right bumpers (L1 & R1) on the controller.
        if (gamepad2.left_bumper) {
            robot.leftClaw.setPosition(0.4);
            robot.rightClaw.setPosition(0.4);
        } else if (gamepad2.right_bumper) {
            robot.leftClaw.setPosition(0.15);
            robot.rightClaw.setPosition(1);
        }



//High basket preset
        if (gamepad2.b && manual == 0.0) {
            robot.lift.setTargetPosition(-9700);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            robot.elbow.setPosition(1);
            robot.wrist.setPosition(1.0);
        }
//Low basket preset
        if (gamepad2.a && manual == 0.0) {
            robot.lift.setTargetPosition(-5000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            robot.elbow.setPosition(1);
            robot.wrist.setPosition(1.0);
        }
//High chamber preset
        if (gamepad2.y && manual == 0.0) {
            robot.lift.setTargetPosition(-8000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            robot.elbow.setPosition(0.385);
            robot.wrist.setPosition(0.23);
        }
//Low chamber preset
        if (gamepad2.x && manual == 0.0){
            robot.lift.setTargetPosition(-4000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            robot.elbow.setPosition(0.385);
            robot.wrist.setPosition(0.23);
        }

        // Digging prese
        if (gamepad2.dpad_down && manual == 0.0){
            robot.elbow.setPosition(0.385);
            robot.wrist.setPosition(0.23);
        }

        //all Grab Prese
        if (gamepad2.dpad_right && manual == 0.0){
            robot.lift.setTargetPosition(-3000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            robot.elbow.setPosition(0.385);
            robot.wrist.setPosition(0.23);
        }



//This code controls the wrist, via the A and Y (X & Triangle) on the controller.
        if (gamepad2.a && manual == 1.0) {
            robot.elbow.setPosition(1);
        }
        else if (gamepad2.y && manual == 1.0) {
            robot.elbow.setPosition(0.2);
        }



//This code controls the wrist, via the B and X (Circle & Square) on the controller.
        if (gamepad2.b && manual == 1.0) {
            robot.wrist.setPosition(1);
        } else if (gamepad2.x && manual == 1.0) {
            robot.wrist.setPosition(0.15);
    }



//This code allows you to move the linear actuator in and out with limits and changeable speed
        if (-gamepad2.right_stick_y > 0 && robot.linear.getCurrentPosition() < 5200) {  // 5030 is upper limit on Linear Actuator for future me
            robot.linear.setVelocity(-gamepad2.right_stick_y * 8000);
        } else if (gamepad2.right_stick_y > 0 && robot.linear.getCurrentPosition() > 30 ) { // 740 is lower limit on Linear Actuator for future me
            robot.linear.setVelocity(-gamepad2.right_stick_y * 9000);
        } else if (gamepad2.share && gamepad2.dpad_down) {   // This code allows you to bypass the limits on the Linear Actuator using the Share button + d-pad up and down
            robot.linear.setPower(-1);                     //
        } else if (gamepad2.share && gamepad2.dpad_up) {     //
            robot.linear.setPower(1);                      //

        } else {
            robot.linear.setPower(0);
        }




//This code allows you to move the lift up and down with limits and changeable speed
        if (-gamepad2.left_stick_y > 0 && robot.lift.getCurrentPosition() > -10000) {  // -5955 is upper limit on lift for future me
            robot.lift.setVelocity(gamepad2.left_stick_y * 2000);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (-gamepad2.left_stick_y < 0 && robot.lift.getCurrentPosition() < -200) {
            robot.lift.setVelocity(gamepad2.left_stick_y * 2000);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }  else if (gamepad2.options && gamepad2.dpad_down) { // This code allows you to bypass the limits on the Lift using the Options button + d-pad up and down
            robot.lift.setPower(-300);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//
        } else if (gamepad2.options && gamepad2.dpad_up) {
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//
            robot.lift.setPower(300);                         //
        } else if (robot.lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.lift.setPower(0);
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

//        telemetry.addData("", "chain %d", robot.chain.getCurrentPosition());
//
            telemetry.addData("", "Lift %d", robot.lift.getCurrentPosition());

            telemetry.addData("", "Linear %d", robot.linear.getCurrentPosition());

            telemetry.addData("", "wrist %f", robot.wrist.getPosition());

            telemetry.addData("", "elbow %f", robot.elbow.getPosition());

            telemetry.addData("", "Left Claw %f   Right Claw %f", robot.leftClaw.getPosition(), robot.rightClaw.getPosition());

            telemetry.addData("", "Driver Speed %f", shift);

            telemetry.addData("", "Manual Mode", manual);

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
            robot.lift.setPower(0);
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


