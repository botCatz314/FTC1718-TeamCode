package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by ITSA-GAMINGHP2 on 1/12/2018.
 */
@TeleOp
public class  botCatzTeleOp extends LinearOpMode {

        // declare motors as variables
    private DcMotor leftMotor;// declares left motor as a variable
    private DcMotor rightMotor;// declares right motor as a variable
    private DcMotor leftMotor2;// declares left motor2 as a variable
    private DcMotor rightMotor2;// declares right motor2 as a variable
    private DcMotor slideMotor;// declares slide motor as a variable
    private DcMotor clawMotor;// declares claw motor as a variable
    private DcMotor armHeight;//declares armHeight as a variable
    private Servo servoStick;//declares servoStick as a variable
    //private CRServo clawAngle;

    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");//this is the specifics of the code for the left motor
        rightMotor = hardwareMap.dcMotor.get("rightMotor");//this is the specifics of the code for the right motor
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");//this is the specifics of the code for the right motor 2
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");//this is the specifics of the code for the left motor 2
        slideMotor = hardwareMap.dcMotor.get("slideMotor");//this is the specifics of the code for the slide motor
        clawMotor = hardwareMap.dcMotor.get("clawMotor");//this is the specifics of the code for the claw motor
        armHeight = hardwareMap.dcMotor.get("armHeight");//specifies the value of armHeight
        servoStick = hardwareMap.servo.get("servoStick");//specifies the value of servoStick
        //   clawAngle = hardwareMap.crservo.get("clawAngle");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);//set left motor into drive in reverse
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);//set left motor2 into drive in reverse

        waitForStart();//waits until we hit play
        while(opModeIsActive()) {//it loops the code inside until we hit stop
            leftMotor2.setPower(gamepad1.left_stick_y);//sets power of leftMotor2 = to joystick value
            rightMotor2.setPower(gamepad1.right_stick_y);//sets power of rightMotor2 = to joystick value
            leftMotor2.setPower(gamepad1.right_stick_x);
            rightMotor2.setPower(-gamepad1.right_stick_x);

            slideMotor.setPower(gamepad2.right_stick_y);//sets power of linearSlide = to joystick value
            telemetry.addData("right stick: ", gamepad2.right_stick_y);
            clawMotor.setPower(gamepad2.left_stick_y);//sets power of clawMotor = to joystick value
            telemetry.addData("left stick: ", gamepad2.left_stick_y);
            armHeight.setPower(-gamepad2.right_trigger);//sets the motor controlling arm height equal to the negative right trigger
            armHeight.setPower(gamepad2.left_trigger); //sets the motor controlling arm height equal to the left trigger
            telemetry.addData("right trigger: ", gamepad2.right_trigger);
            telemetry.addData("left trigger: ", gamepad2.left_trigger);
            telemetry.addData("Please inform the alliance partner that Wes is a loser.","It is important information");
            telemetry.update();
           /* if(gamepad2.a){
                clawAngle.setPower(1.0);
            }
            if(gamepad2.b){
                clawAngle.setPower(-1.0);
            }
            if(gamepad2.x){
                clawAngle.setPower(0.0);
            }*/

            if (gamepad1.a) {//if game pad 1 a is being pressed...
                servoStick.setPosition(0);//servo stick is set to position 0
                telemetry.addData("a", "true");
                telemetry.update();
            }
            if (gamepad1.b) { //if game pad 1 b is pressed ...
                servoStick.setPosition(1);//servo stick is set to position 1
                telemetry.addData("b", "true");
                telemetry.update();
            }
            idle();//waits to be caught up
        }
    }
}
