package org.firstinspires.ftc.teamcode.Utilities;

//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

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
//    public HuskyLens huskyLens;
//    public DcMotorEx hanger;
//    public Servo launcher;
//
//    public Servo Servo1;
//    public Servo Servo2;
//    public DcMotorEx intake;
//    public DcMotorEx lift;
//    public Servo claw;
//    public DcMotor chain;
    public DcMotorEx linear;

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

//        launcher = robotMap.get(Servo.class, "launcher");
//        Servo1 = robotMap.get(Servo.class, "Servo1");
//        Servo2 = robotMap.get(Servo.class, "Servo2");
//        hanger = robotMap.get(DcMotorEx.class, "hanger");
//        claw = robotMap.get(Servo.class, "sticks");
//        lift = robotMap.get(DcMotorEx.class, "lift");
//        intake = robotMap.get(DcMotorEx.class, "intake");
        leftFront  = robotMap.get(DcMotorEx.class, "front_left");
        rightFront = robotMap.get(DcMotorEx.class, "right_front");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear = robotMap.get(DcMotorEx.class, "linear");
//        chain = robotMap.get(DcMotor.class, "chain");
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  = robotMap.get(DcMotorEx.class, "back_left");
        rightBack = robotMap.get(DcMotorEx.class, "right_back");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        huskyLens = robotMap.get(HuskyLens.class, "huskyLens");

        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(IMU.class, "imu");

        // Defines the parameters for the gyro (units)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
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

