package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */

@Autonomous(name = "BlueSide2_v5", group = "Pushbot" )
public class BlueSide_Board2 extends LinearOpMode {
    private DcMotor leftMotor, rightMotor; //Declares the motors
    private Servo servoStickLeft2, servoStickRight1, blockFlicker;
    private ColorSensor colorSensorRight, colorSensorLeft;
    private boolean Left = false, Right = false, Center = false;
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number
    BNO055IMU imu; //declares integrated gyro
    Orientation lastAngle = new Orientation();
    double threshold = .25, colorThreshold;
    double right = 1, left = 0;
    public double pi = 3.1415926535897932;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets properties of gyro from phone
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse
        blockFlicker = hardwareMap.servo.get("blockFlicker");
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

            if(DriveFunctions.LRC() == "Left"){
                Left = true;
                telemetry.addData("left", Left);
                telemetry.update();
            }
            else if (DriveFunctions.LRC() == "Right"){
                Right = true;
                telemetry.addData("right", Right);
                telemetry.update();
            }
            else if(DriveFunctions.LRC() == "Center"){
                Center = true;
                telemetry.addData("Center", Center);
                telemetry.update();
            }
            else{
                Left = true;
                telemetry.addData("None were true ", "going for Left");
            }
            servoStickLeft2.setPosition(1);
            sleep(3000);
            try {
                if (DriveFunctions.ReadColor(colorSensorLeft) == 0) {
                    DriveFunctions.Turn(-0.2, 0.2, leftMotor, rightMotor);
                    sleep(400);
                    DriveFunctions.Brake(leftMotor, rightMotor);
                    servoStickLeft2.setPosition(0);
                    DriveFunctions.Turn(0.2, -0.2, leftMotor, rightMotor);
                    sleep(500);
                    DriveFunctions.Brake(leftMotor,rightMotor);
                }
                else if(DriveFunctions.ReadColor(colorSensorLeft)==1){
                    DriveFunctions.Turn(0.2, -0.2, leftMotor, rightMotor);
                    sleep(400);
                    DriveFunctions.Brake(leftMotor, rightMotor);
                    servoStickLeft2.setPosition(0);
                    DriveFunctions.Turn(-0.2, 0.2, leftMotor, rightMotor);
                    sleep(500);
                    DriveFunctions.Brake(leftMotor, rightMotor);
                }
                else{
                    servoStickLeft2.setPosition(1);
                    sleep(3000);
                }
            }
            catch(IllegalArgumentException INVALIDCOLOR){
                sleep(300);
                servoStickLeft2.setPosition(0);
                sleep(3000);
              //  DriveFunctions.DriveStraight(leftMotor, rightMotor, -0.3);
              //  sleep(4000);
               // DriveFunctions.Brake(leftMotor,rightMotor);
             //   DriveWithEncoders(20, 0.5);
                sleep(2000);
                DriveFunctions.Turn(0.3, -0.3, leftMotor, rightMotor);
                sleep(7000);
                DriveFunctions.Brake(leftMotor, rightMotor);
            }
            servoStickLeft2.setPosition(0);
            sleep(3000);
           // DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
            //DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
            leftMotor.setPower(0.3);
            rightMotor.setPower(0.5);
            sleep(1500);
            DriveFunctions.Brake(leftMotor, rightMotor);
           // DriveFunctions.Brake(leftMotor, rightMotor);
            sleep(100);
            rightMotor.setPower(0.8);
            leftMotor.setPower(-0.2);
            sleep(800);
            DriveFunctions.Brake(leftMotor, rightMotor);

            DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
            sleep(1000);
            DriveFunctions.Brake(leftMotor, rightMotor);

            DriveFunctions.BackUp(leftMotor, rightMotor, 0.4 );
            sleep(400);
            DriveFunctions.Brake(leftMotor, rightMotor);

            blockFlicker.setPosition(0);
            sleep(1000);
            blockFlicker.setPosition(1);
            sleep(1000);
            blockFlicker.setPosition(0);
            sleep(1000);
            blockFlicker.setPosition(1);
            sleep(1000);

            DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
            sleep(1000);
            DriveFunctions.Brake(leftMotor, rightMotor);

            DriveFunctions.BackUp(leftMotor, rightMotor, 1);
            sleep(300);
            DriveFunctions.Brake(leftMotor, rightMotor);


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
            rightMotor.setPower(speed *1.5);

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