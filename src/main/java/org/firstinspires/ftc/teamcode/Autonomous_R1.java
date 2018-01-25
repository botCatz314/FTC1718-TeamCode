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
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2; //Declares the motors
    private Servo servoStick2, servoStick;
    private ColorSensor colorSensor;
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number
    BNO055IMU imu; //declares integrated gyro
    Orientation lastAngle = new Orientation();
    double Kp = 0.35, error, globalAngles;
    public double pi = 3.1415926535897932;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2"); //gets property for right motor from phone
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2"); //gets property of second right motor from phone
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets properties of gyro from phone
        servoStick2 = hardwareMap.servo.get("servoStick2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        servoStick = hardwareMap.servo.get("servoStick");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse
        rightMotor2.setDirection(DcMotor.Direction.REVERSE); //sets the right motors reverse

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


        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart(); //waits until the user presses play
        while (opModeIsActive()) {

            // DriveWithEncoders(20, .5);
        /*    servoStick.setPosition(0);
            sleep(200);

            if(colorSensor.blue()*1.1 > colorSensor.red()){
                DriveWithEncoders(100, .7);

                servoStick.setPosition(1);
            }
            else if(colorSensor.red() *1.1 > colorSensor.blue()){
                DriveWithEncoders(10, -0.4);
                servoStick.setPosition(1);
            }*/

            DriveWithEncoders(20, .5);
            sleep(200);

            rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(100);
            rightMotor2.setPower(0.6);
            leftMotor2.setPower(-0.6);
            sleep(1000);
            rightMotor2.setPower(0);
            leftMotor2.setPower(0);

            sleep(200);
            rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            DriveWithEncoders(-16, 0.2);
            sleep(30000);
        }
    }

    public void DriveWithEncoders(double distance, double speed) {
        double off = 0;
        double encoderCounts = 1120;
        double driveGearReduction = 4.0;
        double wheelDiameter = 9;
        double countsPerMM = (encoderCounts * driveGearReduction) / (wheelDiameter * pi);

        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor2.getCurrentPosition() + (int) (countsPerMM * distance);
            newRightTarget = rightMotor2.getCurrentPosition() + (int) (countsPerMM * distance);

            leftMotor2.setTargetPosition(newLeftTarget);
            rightMotor2.setTargetPosition(newRightTarget);

            leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor2.setPower(speed);
            rightMotor2.setPower(speed);

            while (opModeIsActive() && leftMotor2.isBusy() && rightMotor2.isBusy()) {
                telemetry.addData("target left position: ", newLeftTarget);
                telemetry.addData("target right position: ", newRightTarget);
                telemetry.addData("current left position", leftMotor.getCurrentPosition());
                telemetry.addData("current right position: ", rightMotor.getCurrentPosition());
                telemetry.addData("rightMotor: ", rightMotor.isBusy());
                telemetry.addData("leftMotor", leftMotor.isBusy());
                telemetry.addData("rightSpeed", rightMotor2.getPower());
                telemetry.addData("leftSpeed", leftMotor2.getPower());
                telemetry.update();
            }
            leftMotor2.setPower(off);
            rightMotor2.setPower(off);


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