package org.firstinspires.ftc.teamcode.Utilities;

//import com.qualcomm.hardware.bosch.BNO055IMU i
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Generic robot class
public class GearHoundsHardware extends Hardware {
    public HardwareMap robotMap;

    // Drivetrain Members
    public DcMotorEx leftFront;
    public DcMotorEx  rightFront;
    public DcMotorEx  leftBack;
    public DcMotorEx  rightBack;
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public DcMotorEx arm;
//    public HuskyLens huskyLens;
    public DcMotorEx linear;
    public Servo claw;
    public Servo rotate;
    public Servo wrist;
    public ServoArm UpClawL;
    public Servo UpClawR;

    public static final int READ_PERIOD = 1;
    //public HuskyLens huskyLens;

    //public RevRoboticsCoreHexMotor intake;
    public IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    //private Orientation             lastAngles = new Orientation();
    //private double                  globalAngle;
    private YawPitchRollAngles             lastAngles;
    private static double                  globalAngle;

    // 1000 ticks was roughly 18 in.
    public static final double TICK_PER_INCH = 600.0/12.0;
    public static final double MinPower = 0.3;

    static final double FEET_PER_METER = 3.28084;

    //Constructor
    public GearHoundsHardware(){

    }

    // Override to set actual robot configuration
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        robotMap = hwMap;

        // Define and Initialize Motors for drivetrain
        //huskyLens = robotMap.get(HuskyLens.class, "huskyLens");

        claw = robotMap.get(Servo.class, "claw");
        rotate = robotMap.get(Servo.class, "rotate");
        wrist = robotMap.get(Servo.class,"wrist");
        UpClawL = new ServoArm(robotMap.get(Servo.class,"up_claw_left"));
        UpClawR = robotMap.get(Servo.class,"up_claw_right");

        leftFront  = robotMap.get(DcMotorEx.class, "front_left");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack  = robotMap.get(DcMotorEx.class, "back_left");
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront = robotMap.get(DcMotorEx.class, "front_right");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBack = robotMap.get(DcMotorEx.class, "back_right");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linear = robotMap.get(DcMotorEx.class, "linear");
        linear.setDirection(DcMotorSimple.Direction.REVERSE);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift = robotMap.get(DcMotorEx.class, "left_lift");
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift = robotMap.get(DcMotorEx.class, "right_lift");
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = robotMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        huskyLens = robotMap.get(HuskyLens.class, "huskyLens");

        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(IMU.class, "imu");

        // Defines the parameters for the gyro (units)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(imuParameters);
        resetAngle();
    }
      /*  imuParameters.angleUnit           = IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
*/
    // Intializes the parameters previously defined



    public void resetAngle() {
        imu.resetYaw();
        lastAngles = imu.getRobotYawPitchRollAngles();
        globalAngle = 0;
    }

    public double getAngle() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
//
//    public void resetAngle()
//    {
//        //don't use resetAngle, it messes with the absolute heading
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    public double getAngle()
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
}

