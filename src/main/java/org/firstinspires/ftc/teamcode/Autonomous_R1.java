package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */

@Autonomous(name = "Autonomous_R1", group = "Pushbot" )
public class Autonomous_R1 extends LinearOpMode {
    private DcMotor leftMotor, rightMotor; //Declares the drive motors

    private Servo servoStickLeft2, servoStickRight1, blockFlicker; //declares servos

    private ColorSensor colorSensorLeft, colorSensorRight; //declares color sensors

    BNO055IMU imu; //declares integrated gyro
    Orientation lastAngle = new Orientation();

    private double Kp = 0.35, error, globalAngles, powerOff = 0;
     private double pi = 3.1415926535897932;

    @Override
    public void runOpMode() {

        //declares drive motor
        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone

        //declares sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets properties of gyro from phone
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");

        //declares servos
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");

        //declares attachment motors
        blockFlicker = hardwareMap.servo.get("blockFlicker");

        /*sets parameters*/
        //sets right motors to go the correct direction
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse

        //sets parameters of gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        imu.readCalibrationData(); //calibrates gyro
        imu.isGyroCalibrated(); //checks that gyro is calibrated
        //shows user that gyro is calibrated


        //stops and resets encoders of Drive Motors and runs using encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart(); //waits until the user presses play
        while (opModeIsActive()) {

            DriveWithEncoders(20, .5);
            sleep(200);

            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(100);
            rightMotor.setPower(0.6);
            leftMotor.setPower(-0.6);
            sleep(1000);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            sleep(200);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            DriveWithEncoders(-16, 0.2);
            sleep(30000);
        }
    }

    //declares function that calculates math to drive by encoder
    private void DriveWithEncoders(double distance, double speed) {
        //computes mathematical formulas that translate between encoder counts and real distance
        double off = 0;
        double encoderCounts = 1120;
        double driveGearReduction = 4.0;
        double wheelDiameter = 9;
        double countsPerMM = (encoderCounts * driveGearReduction) / (wheelDiameter * pi);

        //declares two variables to serve as our left and right motor
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            //adds desired distance to the current reading of encoders to ensure accurate measurements
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (countsPerMM * distance);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (countsPerMM * distance);

            //sets the motors' target positions
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            //sets the motors' mode
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //turn the motors on for input speed
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            //loops and gives telemetry until the motors are finished
            while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
                telemetry.addData("target left position: ", newLeftTarget);
                telemetry.addData("target right position: ", newRightTarget);
                telemetry.addData("current left position", leftMotor.getCurrentPosition());
                telemetry.addData("current right position: ", rightMotor.getCurrentPosition());
                telemetry.addData("rightMotor: ", rightMotor.isBusy());
                telemetry.addData("leftMotor", leftMotor.isBusy());
                telemetry.addData("rightSpeed", rightMotor.getPower());
                telemetry.addData("leftSpeed", leftMotor.getPower());
                telemetry.update();
            }

            //turns power of motors off
            leftMotor.setPower(off);
            rightMotor.setPower(off);
        }
    }

    public double readGyro(){
        //gets value of Gyro
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES);
        double deltaAngle = angle.firstAngle -lastAngle.firstAngle; //change in angle = new - old

        if(deltaAngle < 180){
            deltaAngle +=360; //keeps delta angle within valid range
        }
        else if(deltaAngle > 180){
            deltaAngle -= 360; //keeps delta angle within valid range
        }
        globalAngles +=deltaAngle; //global Angle = globalAngle + deltaAngle
        lastAngle = angle; //sets last angle to the angle measurement we just received
        return globalAngles;
    }

    public double CalculateError(double desiredAngle){
        double error;
        error = desiredAngle - readGyro();
        return error;
    }

    public boolean OnHeading(double speed, double angle, double Kp){
        double error, steer, leftSpeed, rightSpeed;
        boolean onTarget = false;
        error = CalculateError(angle);

        return onTarget;
    }
}