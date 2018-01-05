package org.firstinspires.ftc.teamcode;

import android.graphics.Region;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.sun.tools.javac.util.Name;

/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */
@Autonomous(name = "Autonomous_B1", group = "Pushbot" )
public class Autonomous_B1 extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor2;
    private DcMotor rightMotor2;
    private ColorSensor colorSensor;
    double powerOff = 0;
    @Override
    public void runOpMode()
    {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        waitForStart();
    }
    public void driveWithEncoders(double rightAmount, double leftAmount, double speed)
    {
        final double encoderCounts = 560;
        final double driveReduction = 4.0;
        final double wheelDiameter = 90;
        final double countPerMM = (encoderCounts * driveReduction)/(wheelDiameter * 3.14159);
        int rightTarget;
        int leftTarget;
        resetAllEncoders();
        runUsingEncoders();
        if(opModeIsActive())
        {
            rightTarget = rightMotor.getCurrentPosition()+(int)(rightAmount * countPerMM);
            leftTarget = leftMotor.getCurrentPosition()+(int)(leftAmount * countPerMM);
            leftMotor.setTargetPosition(leftTarget);
            leftMotor2.setTargetPosition(leftTarget);
            rightMotor.setTargetPosition(rightTarget);
            rightMotor2.setTargetPosition(rightTarget);

            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            rightMotor2.setPower(speed);
            leftMotor2.setPower(speed);

            while(opModeIsActive() && rightMotor.isBusy() && leftMotor.isBusy()
                    && rightMotor2.isBusy() && leftMotor2.isBusy())
            {
                telemetry.addData("RightTarget: ", rightAmount);
                telemetry.addData("LeftTarget: ", leftAmount);
                telemetry.addData("CurrntPosRight", rightMotor.getCurrentPosition());
                telemetry.addData("CurrentPosLeft", leftMotor.getCurrentPosition());
            }
            leftMotor.setPower(powerOff);
            rightMotor.setPower(powerOff);
            rightMotor2.setPower(powerOff);
            leftMotor2.setPower(powerOff);
            runUsingEncoders();
        }
        resetAllEncoders();
    }
    public void resetAllEncoders()
    {
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders()
    {
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
