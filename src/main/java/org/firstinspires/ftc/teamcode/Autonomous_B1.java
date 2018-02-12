package org.firstinspires.ftc.teamcode;

import android.graphics.Region;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Name;

//import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Time;
import java.util.Iterator;
import java.util.Timer;

/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */


@Autonomous(name = "Autonomous_B1", group = "Pushbot" )
@Disabled
public class Autonomous_B1 extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2; //Declares the motors
    private ColorSensor colorSensor;     //declares the color sensor
    private Servo servoStick; //declares servos
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number
    static final double Kp_Turn = 0.65f; //P constant for PID turns
    static final double Kp_Drive = 0.13f; //P constant for gyro corrective dive
    BNO055IMU imu; //declares integrated gyro

    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2"); //gets property for right motor from phone
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2"); //gets property of second right motor from phone
        servoStick = hardwareMap.servo.get("servoStick"); //gets property of servo for lowering shaft to hit jewel from phone
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor"); //gets property of color sensor from phone
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets properties of gyro from phone
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse
        rightMotor2.setDirection(DcMotor.Direction.REVERSE); //sets the right motors reverse


        //sets parameters of gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        imu.readCalibrationData(); //calibrates gyro
        imu.isGyroCalibrated(); //checks that gyro is
        resetAllEncoders();
        //shows user that gyro is calibrated
        telemetry.addData("gyro: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("You messed up already,", "Wes");
        telemetry.addData("Do you","feel bad yet?");
        telemetry.addData("you","should");
        telemetry.update(); //updates telemetry so user can see that the gyro is calibrate

        waitForStart(); //waits until the user presses play


        telemetry.addData("speed", rightMotor.getPower());
        telemetry.update();

/*
        telemetry.addData("still going", "true");
        telemetry.update();
        sleep(10000);
        imu.readCalibrationData();
        imu.isGyroCalibrated();
        telemetry.addData("calibrate: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);

        servoStick.setPosition(1);
        sleep(3000);

        if(colorSensor.red() > colorSensor.blue()){
            telemetry.addData("WE ARE BLUE ", "HORRAY!");
            telemetry.addData("blue: ", colorSensor.blue());
            telemetry.addData("red: ", colorSensor.red());
            telemetry.update();
            gyroDrive(10,1.0, 0);
            sleep(3000);
            servoStick.setPosition(0);
            sleep(3000);

        }
        else if(colorSensor.blue() > colorSensor.red()){
            telemetry.addData("we are red ", "awww");
            telemetry.addData("blue: ", colorSensor.blue());
            telemetry.addData("red: ", colorSensor.red());
            gyroDrive(-10,1.0, 0);
            sleep(3000);
            servoStick.setPosition(0);
            sleep(3000);
            telemetry.update();
        }
        else{
            telemetry.addData("you failed", "hehhehehehe");
            telemetry.update();
            servoStick.setPosition(0);
        }

        //sleep(3000);*/
        //Erik's code
       /*
        servoStick.setPosition(0); //moves servoStick down so it can hit jewels and reads color
        if (colorSensor.red() > colorSensor.blue()) { //senses whether red is greater than blue
            gyroDrive(100, 1.0, 0); //if red is greater than blue it drives forwards ten cm to knock off jewels
            servoStick.setPosition(1); //resets servoSticks position
            gyroDrive(-65, 1.0, 0); //drives backwards to get near crypto box
        } else if (colorSensor.blue() > colorSensor.red()) { //Senses if blue is greater than red
            gyroDrive(-100, 1.0, 0); //drives backwards to knock ball off jewel stand
            servoStick.setPosition(1); //resets position of servoStick
            gyroDrive(265, 1.0, 0); //Drives backwards to get nearer to crypto box
        }
            gyroTurn(.5,-90); //turns towards crypto box
            gyroDrive(60, .5, 0); //drives to place glyph
            */


    }
    public void resetAllEncoders() //function that allows us to stop and reset the values of all the encoders
    {
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //resets right encoder
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //resets left encoder
    }
    private void runUsingEncoders() //function that tells all motors to run with encoders
    {
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //tells right motor to run with encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //tells left motor to run with encoder
    }

    public double CalculateError(double desiredAngle){ //function that calculates turn and drive error
        double error; //declares error variable
        //solves for error by subtracting the desired angle by the actual angle
        error = desiredAngle - imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle;
        return error; //returns the error
    }
    public double getSteering(double error, double PCoeff){ //gets the amount of change needed
        double priorError = 0; //declares a variable to be the error of the previous loop
        double integral = 0; //declares a variable to use in the I of PID
        long millisInNano = ElapsedTime.MILLIS_IN_NANO; //gets the elapsed time
        double Ki = 0; //I constant
        double derivative; //variable for the Derivative in PID
        double Kd = 0; //D constant
        //solves the derivative part of PID. It subtracts the current error by the previous error and divides the sum by the elapsed time
        derivative = (error - priorError)/millisInNano;
        //solves the integral part of PID by taking the previous integral value and adding it to the product of the elapsed time and the error
        integral = integral + (millisInNano * error);
        //gives telemetry so user can see readings
        telemetry.addData("error: ", error);
        telemetry.addData("p: ", error *PCoeff);
        telemetry.update();
        priorError = error;
        return Range.clip((error *PCoeff)+ (integral *Ki) + (derivative * Kd), -1, 1); //returns PID value
    }
  /*  public void gyroDrive(double distance, double speed, double angle){ //function to move based off the gyro
        int newLeftTarget; //Declares a variable to be the distance that the left motor wants to move
        int newRightTarget; // declares a variable to hold the distance that the left motor wants to move
        int moveCounts; //holds the number of counts the motor needs to go to travel the correct distance
        double max; //variable to keep speed within the speed range
        double error;
        double steer;
        double rightSpeed;
        double leftSpeed;
        final double encoderCounts = 560;
        final double driveReduction = 4.0;
        final double wheelDiameter = 90;
        final Double countsPerMM = (encoderCounts * driveReduction)/(wheelDiameter *3.14159);

        if(opModeIsActive()){
            moveCounts = (int)(distance*countsPerMM);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            telemetry.addData("rightTarget: ", rightMotor.getTargetPosition());
            telemetry.addData("leftTarget: ", leftMotor.getTargetPosition());
            telemetry.update();

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            rightMotor2.setPower(speed);
            leftMotor2.setPower(speed);

            while(opModeIsActive() && rightMotor.isBusy() && leftMotor.isBusy()){

                telemetry.addData("left: ", leftMotor.getPower());
                telemetry.addData("Left Theory: ", speed);
                telemetry.addData("right: ", rightMotor.getPower());
                telemetry.addData("right Theory: ", speed);
                telemetry.addData("right2: ", rightMotor2.getPower());
                telemetry.addData("left2: ", leftMotor2.getPower());
                telemetry.addData("left Target: ", newLeftTarget);
                telemetry.addData("right Target: ", newRightTarget);
                telemetry.update();
                error = CalculateError(angle);
                steer = getSteering(error, Kp_Drive);
                if(distance < 0){
                    steer *= -1;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;
                    telemetry.addData("steer drive: ", steer);
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if(max > 1.0){
                        leftSpeed /= max;
                        rightSpeed /=max;
                    }
                    leftMotor.setPower(leftSpeed);
                    rightMotor.setPower(rightSpeed);
                    rightMotor2.setPower(rightSpeed);
                    leftMotor2.setPower(leftSpeed);

                }
            }
           leftMotor.setPower(powerOff);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           rightMotor.setPower(powerOff);
           rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           rightMotor2.setPower(powerOff);
           rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           leftMotor2.setPower(powerOff);
           leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            runUsingEncoders();
        }
    } */
    private void gyroTurn(double speed, double angle){

        while(opModeIsActive() && !onHeading(speed, angle, Kp_Turn) ){
          //  telemetry.addData("gryro: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("angle: ", angle);
            telemetry.update();
        }
    }
    private boolean onHeading(double speed, double angle, double Kp){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        error = CalculateError(angle);
        if(Math.abs(error) <= 1){
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            telemetry.addData("error: ", error);
            telemetry.update();
        }
        else{
            steer = getSteering(error, Kp);
            rightSpeed = speed *steer;
            leftSpeed = -rightSpeed;
            telemetry.addData("steer turn: ", steer);
            telemetry.update();
        }
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
        rightMotor2.setPower(rightSpeed);
        leftMotor2.setPower(leftSpeed);
        return onTarget;
    }
    public void gyroHold(double speed, float angle, double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while(opModeIsActive() && (holdTimer.time() < holdTime)){
            onHeading(speed, angle, Kp_Turn);
        }
       leftMotor.setPower(powerOff);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightMotor.setPower(powerOff);
       rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightMotor2.setPower(powerOff);
       rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftMotor2.setPower(powerOff);
       leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
