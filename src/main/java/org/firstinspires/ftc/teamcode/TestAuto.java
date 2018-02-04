package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@Autonomous(name = "TestAuto", group = "Pushbot" )
public class TestAuto extends LinearOpMode {
    private DcMotor leftMotor, rightMotor; //Declares the motors
    private Servo servoStickLeft2, servoStickRight1;
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number
    BNO055IMU imu; //declares integrated gyro
    Orientation lastAngle = new Orientation();
    double threshold = .25;
    double right = 1, left = 0;
    public double pi = 3.1415926535897932;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets properties of gyro from phone
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
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

        /*telemetry.addData("gyro", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.update();*/

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart(); //waits until the user presses play
        while (opModeIsActive()) {
/*
   // turnGyro(90,0.5);
    //telemetry.addData("gyro",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES));
   // telemetry.update();

    telemetry.addData("actually running: ", true);
    telemetry.addData("servo left pos:", servoStickLeft2.getPosition());
    telemetry.addData("servo right Pos", servoStickRight1.getPosition());
    telemetry.update();

            servoStickRight1.setPosition(1);
            telemetry.addData("actually running: ", true);
            telemetry.addData("servo left pos:", servoStickLeft2.getPosition());
            telemetry.addData("servo right Pos", servoStickRight1.getPosition());
            telemetry.update();
            sleep(1500);
            servoStickRight1.setPosition(0);

            sleep(30000);
     */
//this is what I was testing last
          /*  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle = 0;
            telemetry.addData("gyro", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            sleep(400);
            turnGyro(90, 0.2);
            sleep(100);
            telemetry.addData("gyro", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            sleep(30000);*/
          servoStickRight1.setPosition(1);
          sleep(3000);
          servoStickRight1.setPosition(0);
          telemetry.addData("We just", "Made it here");
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
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (countsPerMM * distance);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (countsPerMM * distance);

            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

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
            leftMotor.setPower(off);
            rightMotor.setPower(off);


        }
    }
    public double CalculateError(double desiredAngle) {
        double error;
        error = desiredAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("error", error);
        telemetry.update();
        return error;
    }
    private void turnGyro(double angle, double speed){
        telemetry.addData("turn Gyro", angle);
        telemetry.update();
        while(opModeIsActive() && !OnHeading(speed, angle)){
            telemetry.addData("turning", "check");
            telemetry.update();
        }
    }

    public boolean OnHeading(double speed, double angle) {
        double error, steer, leftSpeed, rightSpeed;
        boolean onTarget = false;
        error = CalculateError(angle);
        if(error <= threshold ){
            steer = 0.0;
            leftSpeed= 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = adjustHeading(error);
            rightSpeed = -(speed * steer);
            leftSpeed = -rightSpeed;

        }
        rightMotor.setPower(rightSpeed);
        leftMotor.setPower(leftSpeed);
        return onTarget;
    }

    private double adjustHeading(double error){
       double  Kp = .15, Ki = 0, Kd = 0;
       double errorPrior = 0;
       double integral = 0, derivative = 0;
        ElapsedTime turning = new ElapsedTime();
        turning.reset();
       while(true) {
           integral = integral + (error * turning.time());
           derivative = (error - errorPrior)/turning.time();
           errorPrior = error;
          // sleep(1);
           return Range.clip((error * Kp)+(Ki*integral)+(Kd*derivative), -1, 1);

       }
    }
}