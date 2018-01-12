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

@Autonomous(name = "Autonomous_R1", group = "Concept" )

public class Autonomous_R1_Left extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2; //Declares the motors
    private ColorSensor colorSensor;     //declares the color sensor
    private Servo servoStick; //declares servos
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number
    BNO055IMU imu; //declares integrated gyro

    @Override
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
        imu.isGyroCalibrated(); //checks that gyro is calibrated
        //shows user that gyro is calibrated

        waitForStart(); //waits until the user presses play


        servoStick.setPosition(1);//servo stick down motion
        sleep(2000);//pause code for 2 seconds
        if(colorSensor.blue() > colorSensor.red()){// asking if red's presence is greater than blue
            telemetry.addData("red: ", colorSensor.red());//prints red's value to phone
            telemetry.addData("blue: ", colorSensor.blue());//prints blue's value to phone
            telemetry.update();//prints the telemetry to screen
            Drive(.2);// moves forwars at 40% power
            sleep(500); // pause for 1/2 second
            Brake(); // stops wheel motion
            sleep(500);
            servoStick.setPosition(0); // bring up the jewel stick

        }
        else if(colorSensor.red() > colorSensor.blue() ||true){// if blue is greater than red...
            telemetry.addData("red: ", colorSensor.red());// queues value of red to be printed
            telemetry.addData("blue: ", colorSensor.blue());//queues value of blue to be printed
            telemetry.update();// prints queued data to phone
            Drive_Backwards(.2); // drives backwards at 40% power
            sleep(500); // stops for 1/2 a seconds
            Brake();// stops wheel movement
            servoStick.setPosition(0);// bring up jewel whacker
            Drive(.2);//drives to roughly the same place as other part of if statement
            sleep(1000);//pauses for one second
            Brake();//stops all wheel movement
        }
        servoStick.setPosition(0);// auxiliary bringing up of jewel whacker6
        Drive(.4);
        sleep(1000);
        Brake();
        Turn_Left(.3);
        sleep(500);
        Brake();

        Drive(.3);
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
    private void Drive_Backwards(double power){//reverse all wheels
        leftMotor.setPower(-power);//left motor drives in reverse
        rightMotor.setPower(-power);//right motor drives in reverse
        rightMotor2.setPower(-power);//right motor2 drives in reverse
        leftMotor2.setPower(-power);//left motor2 drives in reverse
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
}