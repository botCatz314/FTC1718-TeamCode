package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * Created by ITSA-GAMINGHP2 on 2/9/2018.
 */
@Autonomous(name = "B2_BCatz_Auto_v3", group = "Pushbot" )
@Disabled
public class B2_BCatz_Auto extends LinearOpMode {
        private DcMotor leftMotor, rightMotor,clawMotor,armHeight; //Declares the drive motors

        private Servo servoStickLeft2, servoStickRight1, blockFlicker; //declares servos

        private ColorSensor colorSensorLeft, colorSensorRight; //declares color sensors
    double threshold = .25, colorThreshold;

    @Override
    public void runOpMode(){
        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        clawMotor = hardwareMap.dcMotor.get("clawMotor");
        armHeight = hardwareMap.dcMotor.get("armHeight");

        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");

        //declares servos
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");

        //declares attachment motors
        blockFlicker = hardwareMap.servo.get("blockFlicker");
        waitForStart(); //waits until the user presses play
        while (opModeIsActive()) {
            telemetry.addData("Running B2_BCatz_Auto", "--");
            telemetry.update();
            servoStickLeft2.setPosition(1);
            sleep(3000);
            try {
                if (DriveFunctions.ReadColor(colorSensorLeft) == 1) {
                    telemetry.addData("Seeing blue", "--");
                    telemetry.update();
                    telemetry.addData("driving straight", "--");
                    telemetry.update();
                    DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.5);
                    sleep(200);
                    DriveFunctions.Brake(leftMotor, rightMotor);
                    telemetry.addData("Servo Up", "--");
                    telemetry.update();
                    servoStickLeft2.setPosition(0);
                    sleep(3000);
                    telemetry.addData("Jewel Functions Done", "--");
                    telemetry.update();
                }
                else if(DriveFunctions.ReadColor(colorSensorLeft)==0){
                    telemetry.addData("Seeing red", "--");
                    telemetry.update();
                    telemetry.addData("Reversing", "--");
                    telemetry.update();
                    DriveFunctions.DriveStraight(leftMotor, rightMotor, -0.3);
                    sleep(500);
                    DriveFunctions.Brake(leftMotor, rightMotor);
                    sleep(100);
                    telemetry.addData("Servo Up", "--");
                    telemetry.update();
                    servoStickLeft2.setPosition(0);
                    sleep(3000);
                    telemetry.addData("Driving Straight", "--");
                    telemetry.update();
                    DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
                    sleep(2500);
                    DriveFunctions.Brake(leftMotor, rightMotor);
                    telemetry.addData("Servo Up", "--");
                    telemetry.update();
                    servoStickLeft2.setPosition(0);
                    sleep(3000);
                }
                else{
                    telemetry.addData("IDK?", "--");
                    telemetry.update();
                    servoStickLeft2.setPosition(0);
                    sleep(3000);
                    telemetry.addData("Servo Up", "--");
                    telemetry.update();
                    DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
                    sleep(2000);
                    telemetry.addData("Driving Straight", "--");
                    telemetry.update();
                    DriveFunctions.Brake(leftMotor, rightMotor);
                }
            }
            catch(IllegalArgumentException INVALIDCOLOR){
                sleep(300);
                servoStickLeft2.setPosition(0);
                sleep(3000);
                DriveFunctions.DriveStraight(leftMotor, rightMotor, 0.3);
                sleep(2000);
                DriveFunctions.Brake(leftMotor,rightMotor);
            }

            sleep(30000);
        }

    }
}
