package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@TeleOp(name="LiftTest", group= "org/firstinspires/ftc/teamcode/TeleOp")
public class LiftTest extends OpMode {
    // FTC Dashboard instance
    private FtcDashboard dashboard;
    
    // Dashboard configurable variables
    public static double LIFT_KP = 0.012;  // Proportional gain for lift
    public static double LIFT_KI = 0.0001; // Integral gain for lift
    public static double LIFT_KD = 0.001;  // Derivative gain for lift
    
    public static double ARM_KP = 0.015;   // Proportional gain for arm
    public static double ARM_KI = 0.0001;  // Integral gain for arm
    public static double ARM_KD = 0.001;   // Derivative gain for arm
    
    // Motor limits
    public static int LIFT_UPPER_LIMIT = -3500;  // Upper limit for lift
    public static int LIFT_LOWER_LIMIT = 0;      // Lower limit for lift
    public static int ARM_UPPER_LIMIT = 1000;    // Upper limit for arm
    public static int ARM_LOWER_LIMIT = -1000;   // Lower limit for arm
	
	//Limit Brake power limits
	public static double LIFT_MIN_BRAKE_POWER = 0.05;  // Minimum power to hold position
	public static double LIFT_MAX_BRAKE_POWER = 0.2;   // Maximum brake power
	public static double LIFT_BRAKE_GAIN = 0.001;      // How aggressively to brake (adjust as needed)
	public static double ARM_MIN_BRAKE_POWER = 0.05;  // Minimum power to hold arm position
	public static double ARM_MAX_BRAKE_POWER = 0.2;   // Maximum arm brake power
	public static double ARM_BRAKE_GAIN = 0.001;      // How aggressively to brake arm (adjust as needed)
    
    // Servo positions
    public static double CLAW_OPEN_POS = 0.624;
    public static double CLAW_CLOSED_POS = 0.67;
    public static double ROTATE_SIDEWAYS_POS = 0.8;
    public static double ROTATE_FORWARD_POS = 0.485;
    public static double WRIST_UP_POS = 0.845;
    public static double WRIST_DOWN_POS = 0;
    public static double UPCLAW_L_OPEN = 0.28;
    public static double UPCLAW_L_CLOSED = 0;
    public static double UPCLAW_R_OPEN = 0.3;
    public static double UPCLAW_R_CLOSED = 0;

    // Declare the hardwareMap for the robot
    private GearHoundsHardware robot = new GearHoundsHardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    // PID variables for lift
    private double liftIntegral = 0;
    private double lastLiftError = 0;
    private double liftTargetPosition = 0;
    
    // PID variables for arm
    private double armIntegral = 0;
    private double lastArmError = 0;
    private double armTargetPosition = 0;
    
    // Variables for tracking last update time for PID
    private double lastTime = 0;
    
    // Drivetrain control variables
    private double shift = 1.0;
    private double P1xTime = -10;
    private double P1aTime = -10;
    private double P1lpadTime = -10;
    private double P1rpadTime = -10;

    // State tracking variables
    boolean Moving = false;
    boolean Chaining = false;
    boolean Handing = false;
    boolean ClawOpen = false;
    boolean ClawSideways = false;

    @Override
    public void init() {
        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        
        // Initialize the hardwareMap
        robot.init(hardwareMap);
        
        // Configure lift motors for PID control
        robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize PID timer
        lastTime = runtime.seconds();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Robot Initialized!");
        telemetry.update();
        
        // Dashboard telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Status", "Initialized");
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        runtime.reset();
        gamepad1.setLedColor(17, 0, 17, 1000000000);
        gamepad2.setLedColor(0, 10, 10, 1000000000);
        shift = 1.0;
    }

    // PID control function for lift system
    private double calculateLiftPID(double targetPosition, double currentPosition) {
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;
        
        // Calculate error
        double error = targetPosition - currentPosition;
        
        // Calculate integral term with anti-windup
        liftIntegral = liftIntegral + (error * deltaTime);
        if (Math.abs(liftIntegral) > 1000) {
            liftIntegral = 1000 * Math.signum(liftIntegral);
        }
        
        // Calculate derivative term
        double derivative = (error - lastLiftError) / deltaTime;
        
        // Calculate PID output using dashboard values
        double output = (LIFT_KP * error) + (LIFT_KI * liftIntegral) + (LIFT_KD * derivative);
        
        // Update last values for next iteration
        lastLiftError = error;
        
        return Math.max(-1, Math.min(1, output));
    }

    // PID control function for arm system
    private double calculateArmPID(double targetPosition, double currentPosition) {
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;
        
        // Calculate error
        double error = targetPosition - currentPosition;
        
        // Calculate integral term with anti-windup
        armIntegral = armIntegral + (error * deltaTime);
        if (Math.abs(armIntegral) > 1000) {
            armIntegral = 1000 * Math.signum(armIntegral);
        }
        
        // Calculate derivative term
        double derivative = (error - lastArmError) / deltaTime;
        
        // Calculate PID output using dashboard values
        double output = (ARM_KP * error) + (ARM_KI * armIntegral) + (ARM_KD * derivative);
        
        // Update last values for next iteration
        lastArmError = error;
        
        return Math.max(-1, Math.min(1, output));
    }

    @Override
    public void loop() {
        // Update timestamp for PID calculations
        lastTime = runtime.seconds();
        
        // Create dashboard packet
        TelemetryPacket packet = new TelemetryPacket();
        
	  // Lift Manual Control with PID and dashboard limits
	if (-gamepad2.left_stick_y > 0 && robot.leftLift.getCurrentPosition() > LIFT_UPPER_LIMIT) {
		liftTargetPosition += (-gamepad2.left_stick_y * 20);
		// Ensure target stays within limits
		liftTargetPosition = Math.max(LIFT_UPPER_LIMIT, Math.min(LIFT_LOWER_LIMIT, liftTargetPosition));
		double liftPower = calculateLiftPID(liftTargetPosition, robot.leftLift.getCurrentPosition());
		robot.leftLift.setPower(liftPower);
		robot.rightLift.setPower(liftPower);
	} else if (-gamepad2.left_stick_y < 0 && robot.leftLift.getCurrentPosition() < LIFT_LOWER_LIMIT) {
		liftTargetPosition += (-gamepad2.left_stick_y * 20);
		liftTargetPosition = Math.max(LIFT_UPPER_LIMIT, Math.min(LIFT_LOWER_LIMIT, liftTargetPosition));
		double liftPower = calculateLiftPID(liftTargetPosition, robot.leftLift.getCurrentPosition());
		robot.leftLift.setPower(liftPower);
		robot.rightLift.setPower(liftPower);
	} else {
		 // Only apply brake if we're not at the lower limit
		double currentPosition = robot.leftLift.getCurrentPosition();
		if (currentPosition <= (LIFT_LOWER_LIMIT + 20)) {  // Adding small threshold
			// At bottom - no brake needed
			robot.leftLift.setPower(0);
			robot.rightLift.setPower(0);
		} else {
			// Brake mode - apply holding power based on position error
			double positionError = liftTargetPosition - currentPosition;
			// Apply brake power proportional to error, with minimum holding power
			double brakePower = Math.max(LIFT_MIN_BRAKE_POWER, 
									   Math.min(LIFT_MAX_BRAKE_POWER, 
									   Math.abs(positionError) * LIFT_BRAKE_GAIN)) 
							   * Math.signum(positionError);
			
			robot.leftLift.setPower(brakePower);
			robot.rightLift.setPower(brakePower);
		}
	}


	// Arm Manual Control with PID and dashboard limits
	if (gamepad2.dpad_up && robot.arm.getCurrentPosition() < ARM_UPPER_LIMIT) {
		armTargetPosition += 10;
		armTargetPosition = Math.max(ARM_LOWER_LIMIT, Math.min(ARM_UPPER_LIMIT, armTargetPosition));
		double armPower = calculateArmPID(armTargetPosition, robot.arm.getCurrentPosition());
		robot.arm.setPower(armPower);
	} else if (gamepad2.dpad_down && robot.arm.getCurrentPosition() > ARM_LOWER_LIMIT) {
		armTargetPosition -= 10;
		armTargetPosition = Math.max(ARM_LOWER_LIMIT, Math.min(ARM_UPPER_LIMIT, armTargetPosition));
		double armPower = calculateArmPID(armTargetPosition, robot.arm.getCurrentPosition());
		robot.arm.setPower(armPower);
	} else {
		// Only apply brake if we're not at the lower limit
		double currentPosition = robot.arm.getCurrentPosition();
		if (currentPosition <= (ARM_LOWER_LIMIT + 20)) {  // Adding small threshold
			// At bottom - no brake needed
			robot.arm.setPower(0);
		} else {
			// Brake mode - apply holding power based on position error
			double positionError = armTargetPosition - currentPosition;
			// Apply brake power proportional to error, with minimum holding power
			double brakePower = Math.max(ARM_MIN_BRAKE_POWER, 
									   Math.min(ARM_MAX_BRAKE_POWER, 
									   Math.abs(positionError) * ARM_BRAKE_GAIN)) 
							   * Math.signum(positionError);
			
			robot.arm.setPower(brakePower);
		}
	}

        // Preset Position Control (Right D-pad)
        if (gamepad2.dpad_right) {
            liftTargetPosition = LIFT_UPPER_LIMIT / 2;  // 50% of max height
            armTargetPosition = (ARM_UPPER_LIMIT + ARM_LOWER_LIMIT) / 2;  // 50% of arm range
            
            double liftPower = calculateLiftPID(liftTargetPosition, robot.leftLift.getCurrentPosition());
            double armPower = calculateArmPID(armTargetPosition, robot.arm.getCurrentPosition());
            
            robot.leftLift.setPower(liftPower);
            robot.rightLift.setPower(liftPower);
            robot.arm.setPower(armPower);
        }

        // Upper claw controls with dashboard positions
        if (gamepad2.ps) {
            robot.UpClawL.setPosition(UPCLAW_L_CLOSED);
            robot.UpClawR.setPosition(UPCLAW_R_CLOSED);
        }

        if (gamepad2.left_bumper) {
            robot.UpClawL.setPosition(UPCLAW_L_CLOSED);
            robot.UpClawR.setPosition(0.65);
        }

        if (gamepad2.right_bumper) {
            robot.UpClawL.setPosition(UPCLAW_L_OPEN);
            robot.UpClawR.setPosition(UPCLAW_R_OPEN);
        }

        // Claw controls with dashboard positions
        if (gamepad1.right_trigger > 0.8) {
            robot.claw.setPosition(CLAW_CLOSED_POS);
            ClawOpen = false;
        }
        if (gamepad1.left_trigger > 0.8) {
            robot.claw.setPosition(CLAW_OPEN_POS);
            ClawOpen = true;
        }

        // Rotation and wrist controls with dashboard positions
        if (gamepad1.dpad_left) {
            P1lpadTime = runtime.seconds();
        }
        if (((runtime.seconds() - P1lpadTime) < 0.03) && ClawOpen) {
            robot.claw.setPosition(CLAW_CLOSED_POS);
        } else if ((runtime.seconds() - P1lpadTime) < 0.04) {
            robot.rotate.setPosition(ROTATE_SIDEWAYS_POS);
            ClawOpen = false;
            ClawSideways = true;
        }

        if (gamepad1.dpad_right) {
            P1rpadTime = runtime.seconds();
        }
        if (((runtime.seconds() - P1rpadTime) < 0.05) && ClawOpen) {
            robot.claw.setPosition(CLAW_CLOSED_POS);
        } else if ((runtime.seconds() - P1rpadTime) < 0.06) {
            robot.rotate.setPosition(ROTATE_FORWARD_POS);
            ClawOpen = false;
            ClawSideways = false;
        }

        // Update dashboard telemetry
        packet.put("Lift Position", robot.leftLift.getCurrentPosition());
        packet.put("Lift Target", liftTargetPosition);
        packet.put("Arm Position", robot.arm.getCurrentPosition());
        packet.put("Arm Target", armTargetPosition);
        packet.put("Lift Power", robot.leftLift.getPower());
        packet.put("Arm Power", robot.arm.getPower());
        dashboard.sendTelemetryPacket(packet);

        // Regular telemetry updates
        telemetry.addData("Left Front", robot.leftFront.getCurrentPosition());
        telemetry.addData("Right Front", robot.rightFront.getCurrentPosition());
        telemetry.addData("Left Back", robot.leftBack.getCurrentPosition());
        telemetry.addData("Right Back", robot.rightBack.getCurrentPosition());
        telemetry.addData("Angle", robot.getAngle());
        telemetry.addData("Linear", robot.linear.getCurrentPosition());
        telemetry.addData("Left Lift", robot.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift", robot.rightLift.getCurrentPosition());
        telemetry.addData("Driver Speed", shift);
        telemetry.update();

        // Mecanum drive code
        double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        
        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
        rotX = rotX * 1.1;
        double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

        double d = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double lf = (rotY + rotX + rx) / d;
        double lb = (rotY - rotX + rx) / d;
        double rf = (rotY - rotX - rx) / d;
		double rb = (rotY + rotX - rx) / d;

        // Set motor velocities for drivetrain
        robot.leftFront.setVelocity(2000 * lf * shift);
        robot.leftBack.setVelocity(2000 * lb * shift);
        robot.rightFront.setVelocity(2000 * rf * shift);
        robot.rightBack.setVelocity(2000 * rb * shift);

        // Update drivetrain telemetry on dashboard
        packet.put("Left Front Velocity", robot.leftFront.getVelocity());
        packet.put("Right Front Velocity", robot.rightFront.getVelocity());
        packet.put("Left Back Velocity", robot.leftBack.getVelocity());
        packet.put("Right Back Velocity", robot.rightBack.getVelocity());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        // Stop all motors
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftLift.setPower(0);
        robot.rightLift.setPower(0);
        robot.arm.setPower(0);
        
        // Final dashboard update
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Status", "Stopped");
        dashboard.sendTelemetryPacket(packet);
    }
}