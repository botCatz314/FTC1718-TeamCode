package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */
@Autonomous(name = "Autonomous_R2", group = "Concept" )
public class Autonomous_R2_Right extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor2;
    private DcMotor rightMotor2;
    private DcMotor slideMotor;
    private DcMotor clawMotor;
    private DcMotor clawAngle;
    private DcMotor clawHeight;
    private Servo servoStick;
    private ColorSensor colorSensor;     //declares the color sensor
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        clawAngle = hardwareMap.dcMotor.get("clawAngle");
        clawHeight = hardwareMap.dcMotor.get("armHeight");
        clawMotor = hardwareMap.dcMotor.get("clawMotor");
        servoStick = hardwareMap.servo.get("servoStick");
      //  colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor"); //gets property of color sensor from phone
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse
        rightMotor2.setDirection(DcMotor.Direction.REVERSE); //sets the right motors reverse


        waitForStart(); //waits until the user presses play


        servoStick.setPosition(1);//servo stick down motion
        sleep(2000);//pause code for 2 seconds
        if(colorSensor.blue() > colorSensor.red()){// asking if red's presence is greater than blue
            Drive(.2);// moves forwars at 40% power
            sleep(500); // pause for 1/2 second
            Brake(); // stops wheel motion
            servoStick.setPosition(0); // bring up the jewel stick
            Drive(.4);//drives to roughly the same place as other part of if statement
            sleep(2000);//pauses for one second
            Brake();//stops all wheel movement
            Turn_Right(0.25);
            sleep(2000);
        }
        else if(colorSensor.red() > colorSensor.blue() || true){// if blue is greater than red...
            Drive(-.2);// moves forwars at 40% power
            sleep(500); // pause for 1/2 second
            Brake(); // stops wheel motion
            servoStick.setPosition(0); // bring up the jewel stick
            Drive(.4);//drives to roughly the same place as other part of if statement
            sleep(2500);//pauses for one second
            Brake();//stops all wheel movement
            Turn_Right(0.25);
            sleep(2000);
            Brake();
        }
        Turn_Right(0.2);
        sleep(1000);
        Brake();
        Drive(0.5);
        sleep(500);
        Brake();
        Release_Glyph();
        Drive(-0.2);
        clawAngle.setPower(0.2);
        sleep(1000);
        Brake();


    }
    private void Drive(double power){//function for driving forwards
        leftMotor.setPower(power);//this turns on the left motor=to power input
        rightMotor.setPower(power);//this turns on the right motor=to power input
        rightMotor2.setPower(power);// this turns on the right motor2=to power input
        leftMotor2.setPower(power);//this turns on the left motor2=to power input
    }
    private void Brake(){//stops all wheel movement
        leftMotor.setPower(0);//brakes left motor
        rightMotor.setPower(0.0);//brakes right motor
        rightMotor2.setPower(0.0);//brakes right motor2
        leftMotor2.setPower(0.0);//brakes left motor2
    }


    private void Turn_Right(double power){//drives motors to turn right
        leftMotor.setPower(power);//drives left motor forwards
        rightMotor.setPower(-power);//drives left motor backwards
        rightMotor2.setPower(-power);//drives right motor2 backwards
        leftMotor2.setPower(power);//drives left motor2 forwards
    }
    private void Turn_Left (double power){//drives all motors left
        leftMotor.setPower(-power);//drives left motor backwards
        rightMotor.setPower(power);//drives right motor forwards
        rightMotor2.setPower(power);//drives right motor2 forwards
        leftMotor2.setPower(-power);//drives left motor2 backwards
    }
    private void Release_Glyph () {
        clawHeight.setPower(0.4);
        sleep(2300);
        clawHeight.setPower(powerOff);
        slideMotor.setPower(0.2);
        sleep(400);
        slideMotor.setPower(powerOff);
        clawAngle.setPower(0.4);
        sleep(1000);
        clawMotor.setPower(0.4);
        sleep(1500 );
        clawMotor.setPower(powerOff);
        clawAngle.setPower(powerOff);

    }
}