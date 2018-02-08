package org.firstinspires.ftc.teamcode;

/**
 * Created by ITSA-GAMINGHP2 on 2/7/2018.
 */
import com.qualcomm.robotcore.hardware.ColorSensor;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
public class DriveFunctions {
    static double colorThreshold =20;
    static public Integer ReadColor(ColorSensor colorSensor){
        if(colorSensor.red() - colorSensor.blue() > colorThreshold ){
            return 0;
        }
        else if(colorSensor.blue()-colorSensor.red() > colorThreshold){
            return 1;
        }
       else{
             IllegalArgumentException INVALIDCOLOR;
             return 2;
        }
    }

    static void DriveStraight(DcMotor left, DcMotor right, double speed){
        left.setPower(speed);
        right.setPower(speed);
    }
    static void Brake(DcMotor left, DcMotor right){
        double stop = 0;
        left.setPower(stop);
        right.setPower(stop);
    }
static public void Turn(double leftsSpeed, double rightSpeed, DcMotor leftMotor, DcMotor rightMotor){
        leftMotor.setPower(leftsSpeed);
        rightMotor.setPower(rightSpeed);
}








}
