package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ITSA-GAMINGHP2 on 1/12/2018.
 */
@TeleOp
public class botCatzTeleOp_SingleStick extends LinearOpMode {

    private float x, y, z, w, pwr;
    private static double deadzone = 0.2;

    // declare motors as variables
    private DcMotor leftMotor, rightMotor; //declares drive motors
    private DcMotor slideMotor, armHeight, slideReverse; //declares attachment motors

    //declares motors as variables
    private Servo servoStickRight1,servoStickLeft2; //declares servos with regular range of motion
    private CRServo clawServo; //declares Continuous Rotational servos

    //private CRServo clawAngle; 
    public void runOpMode() {

        //defines drive motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        //defines attachment motors
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        armHeight = hardwareMap.dcMotor.get("armHeight");
        slideReverse = hardwareMap.dcMotor.get("slideReverse");

        //defines servos
        clawServo = hardwareMap.crservo.get("clawServo");
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");

        //sets parameters
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        slideReverse.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();//waits until we hit play

        while(opModeIsActive()) {//it loops the code inside until we hit stop
            getJoyVals();
            pwr = y; //this can be tweaked for exponential power increase
            //sets power of drive motors
            rightMotor.setPower(Range.clip(pwr - x+z, -1, 1));
            leftMotor.setPower(Range.clip(pwr - x-z, -1, 1));
            //returns data to phones
            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.addData("pwr: ", pwr);
            telemetry.update();

            //controls the linearSlide
            slideMotor.setPower(gamepad2.right_stick_y);
            if(gamepad2.right_stick_y > 1.0){
                gamepad2.right_stick_y = 1.0f;
                slideReverse.setPower(0.5);
            }
            else if(gamepad2.right_stick_y < -0.5){
                gamepad2.right_stick_y = -0.5f;
                slideReverse.setPower(-1.0);
            }

            //sets power of clawServo
            clawServo.setPower(gamepad2.left_stick_y);

            //controls the height of the linear slide
            armHeight.setPower(-gamepad2.right_trigger);//sets the motor controlling arm height equal to the negative right trigger
            armHeight.setPower(gamepad2.left_trigger); //sets the motor controlling arm height equal to the left trigger

            //gives drivers control of the servoStick in case it does not pull up
            if (gamepad1.a) {
                servoStickRight1.setPosition(0);
                telemetry.addData("a", "true");
                telemetry.update();
            }
            if (gamepad1.b) {
                servoStickRight1.setPosition(1);
                telemetry.addData("b", "true");
                telemetry.update();
            }
            if (gamepad1.x) {
                servoStickLeft2.setPosition(0);
                telemetry.addData("Correct", "true");
                telemetry.update();
            }
            if (gamepad1.y) {
                servoStickLeft2.setPosition(1);
                telemetry.addData("Wrong", "true");
                telemetry.update();
            }

            idle();//waits to be caught up
        }
    }
    private void getJoyVals()
    {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        z = gamepad1.left_stick_x;
        w = gamepad1.right_stick_y;
        //updates joystick values

        if(Math.abs(x)<deadzone) x = 0;
        if(Math.abs(y)<deadzone) y = 0;
        if(Math.abs(z)<deadzone) z = 0;
        if(Math.abs(w)<0.9) w = 0;
        //checks deadzones
    }
}
