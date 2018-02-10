package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "TestErik", group = "Pushbot" )
public class TestErik extends LinearOpMode {
    private DcMotor leftMotor, rightMotor; //Declares the motors
    private Servo servoStickLeft2, servoStickRight1,blockFlicker;
    private ColorSensor colorSensorRight, colorSensorLeft;
    private int servoStickUp = 0;
    private int servoStickDown = 1;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        blockFlicker = hardwareMap.colorSensor.get("blockFlicker");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse
        //sets parameters of gyro

        waitForStart(); //waits until the user presses play
        while (opModeIsActive()) {
servoStickRight1.setPosition(servoStickDown);
if (DriveFunctions.ReadColor(colorSensorRight == 0)){
    telemetry.addData("blue","is the color");
    telemetry.update();
    DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
    sleep(300);
    DriveFunctions.Brake(leftMotor,rightMotor);
    servoStickRight1.setPosition(servoStickUp);
    DriveFunctions.BackUp(leftMotor,rightMotor,0.3);
    sleep(2000);
    DriveFunctions.Brake(leftMotor,rightMotor);
}
else if (DriveFunctions.ReadColor(colorSensorRight == 1)){
    telemetry.addData("red","is the color");
    telemetry.update();
    DriveFunctions.BackUp(leftMotor,rightMotor,0.3);
    sleep(1750);
    DriveFunctions.Brake(leftMotor,rightMotor);
}
else {
    servoStickRight1.setPosition(servoStickUp);
}
servoStickRight1.setPosition(servoStickUp);
DriveFunctions.Turn(.3,-.3,leftMotor,rightMotor);
sleep(700);
DriveFunctions.Brake(leftMotor,rightMotor);
blockFlicker.setPosition(1);
            sleep(30000);
        }
    }
}