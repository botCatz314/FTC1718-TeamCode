package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */

@Autonomous(name = "Autonomous_Greeting_v1", group = "Pushbot" )

public class Autonomous_Greeting extends LinearOpMode {
    private DcMotor leftMotor, rightMotor,clawMotor,armHeight; //Declares the drive motors

    private Servo servoStickLeft2, servoStickRight1, blockFlicker; //declares servos

    private ColorSensor colorSensorLeft, colorSensorRight; //declares color sensors



    double down = 1;
    double up = 0;

    // Runs at init time
    @Override
    public void runOpMode() {

        //declares drive motor
        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        clawMotor = hardwareMap.dcMotor.get("clawMotor");
        armHeight = hardwareMap.dcMotor.get("armHeight");

        //declares sensors
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");

        //declares servos
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");

        //declares attachment motors
        blockFlicker = hardwareMap.servo.get("blockFlicker");

        /*sets parameters*/
        //sets right motors to go the correct direction
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//sets the right motors reverse

        // Everything above is the init.
        // This blocks and everything below is after start
        waitForStart(); //waits until the user presses play

        // opModeIsActive will return false when the user hits stop.
        while (opModeIsActive()) {

            log("Begin greeting procedure");

            log("Spin Right!");
            DriveFunctions.Turn(1.0,-1.0, leftMotor,rightMotor);
            sleep(3000);
            DriveFunctions.Brake(leftMotor,rightMotor);

            sleep(500);

            log("Spin Left!");
            DriveFunctions.Turn(-1.0,1.0, leftMotor,rightMotor);
            sleep(3000);
            DriveFunctions.Brake(leftMotor,rightMotor);

            sleep(2000);

            log("Jig Right!");
            DriveFunctions.Turn(1.0,-1.0, leftMotor,rightMotor);
            sleep(300);
            DriveFunctions.Brake(leftMotor,rightMotor);

            sleep(500);

            log("Jig Left!");
            DriveFunctions.Turn(-1.0,1.0, leftMotor,rightMotor);
            sleep(300);
            DriveFunctions.Brake(leftMotor,rightMotor);

            log("Raise arm!");
            armHeight.setPower(-0.4);
            sleep(2000);
            armHeight.setPower(0);

            log("open claw!");
            clawMotor.setPower(0.5);
            sleep(1000);
            clawMotor.setPower(-0.5);
            sleep(1000);
            clawMotor.setPower(0.5);
            sleep(1000);
            clawMotor.setPower(-0.5);
            sleep(1000);
            clawMotor.setPower(0);

            log("Lower arm!");
            armHeight.setPower(0.4);
            sleep(1500);
            armHeight.setPower(0);



            sleep(300000);

        }

    }



    public void FlickBlock(){
        blockFlicker.setPosition(down);
        sleep(1000);
        blockFlicker.setPosition(up);
        sleep(1000);

    }

    public void log (String text1, String text2){
        telemetry.addData(text1, text2);
        telemetry.update();
    }
    public void log (String text1){
        telemetry.addData(text1, "");
        telemetry.update();
    }

    // back forth
    public void BackForth(DcMotor left, DcMotor right, double speed, long t){

        DriveFunctions.BackUp(left,right,speed);
        sleep(t);
        DriveFunctions.Brake(left,right);
        DriveFunctions.DriveStraight(left,right,speed);
        sleep(t);
        DriveFunctions. Brake(left,right);
    }


}
