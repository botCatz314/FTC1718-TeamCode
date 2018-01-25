package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ITSA-GAMINGHP2 on 1/12/2018.
 */
@TeleOp
public class botCatzTeleOp_SingleStick extends LinearOpMode {

    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;

    // declare motors as variables
    private DcMotor leftMotor;// declares left motor as a variable
    private DcMotor rightMotor;// declares right motor as a variable
    private DcMotor leftMotor2;// declares left motor2 as a variable
    private DcMotor rightMotor2;// declares right motor2 as a variable
    private DcMotor slideMotor;// declares slide motor as a variable
    private DcMotor clawMotor;// declares claw motor as a variable
    private DcMotor armHeight;//declares armHeight as a variable
    private Servo servoStickRight1;//declares servoStick as a variable
    private Servo servoStickLeft2;

    private DcMotor slideReverse;
    //private CRServo clawAngle; \
    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");//this is the specifics of the code for the left motor
        rightMotor = hardwareMap.dcMotor.get("rightMotor");//this is the specifics of the code for the right motor
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");//this is the specifics of the code for the right motor 2
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");//this is the specifics of the code for the left motor 2
        slideMotor = hardwareMap.dcMotor.get("slideMotor");//this is the specifics of the code for the slide motor
        clawMotor = hardwareMap.dcMotor.get("clawMotor");//this is the specifics of the code for the claw motor
        armHeight = hardwareMap.dcMotor.get("armHeight");//specifies the value of armHeight
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");//specifies the value of servoStick
        slideReverse = hardwareMap.dcMotor.get("slideReverse");
        servoStickLeft2 = hardwareMap.dcMotor.get("servoStickLeft2");
        //   clawAngle = hardwareMap.crservo.get("clawAngle");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);//set left motor into drive in reverse
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);//set left motor2 into drive in reverse
        slideReverse.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();//waits until we hit play
        while(opModeIsActive()) {//it loops the code inside until we hit stop
            /*leftMotor.setPower(gamepad1.left_stick_y);//sets power of leftMotor = to joystick value
            leftMotor2.setPower(gamepad1.left_stick_y);//sets power of leftMotor2 = to joystick value
            rightMotor.setPower(gamepad1.right_stick_y);//sets power of rightMotor = to joystick value
            rightMotor2.setPower(gamepad1.right_stick_y);//sets power of rightMotor2 = to joystick value
            */

            // Different joystick controls:
            getJoyVals();
            pwr = y; //this can be tweaked for exponential power increase
            rightMotor.setPower(Range.clip(pwr - x+z, -1, 1));
            leftMotor.setPower(Range.clip(pwr - x-z, -1, 1));
            leftMotor2.setPower(Range.clip(pwr + x-z, -1, 1));
            rightMotor2.setPower(Range.clip(pwr + x+z, -1, 1));
            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.addData("pwr: ", pwr);
            //telemetry.update();
            // end different controls

            slideMotor.setPower(gamepad2.right_stick_y);//sets power of linearSlide = to joystick value
            if(gamepad2.right_stick_y > 1.0){
                gamepad2.right_stick_y = 1.0f;
                slideReverse.setPower(0.5);
            }
            else if(gamepad2.right_stick_y < -0.5){
                gamepad2.right_stick_y = -0.5f;
                slideReverse.setPower(-1.0);
            }
            clawMotor.setPower(gamepad2.left_stick_y);//sets power of clawMotor = to joystick value
            armHeight.setPower(-gamepad2.right_trigger);//sets the motor controlling arm height equal to the negative right trigger
            armHeight.setPower(gamepad2.left_trigger); //sets the motor controlling arm height equal to the left trigger
            telemetry.addData("left stick: ", gamepad2.left_stick_y);
            telemetry.addData("right stick: ", gamepad2.right_stick_y);
            telemetry.addData("right trigger: ", gamepad2.right_trigger);
            telemetry.addData("left trigger: ", gamepad2.left_trigger);
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
                servoStickRight1.setPosition(0);//servo stick is set to position 0
                telemetry.addData("a", "true");
                telemetry.update();
            }
            if (gamepad1.b) { //if game pad 1 b is pressed ...
                servoStickRight1.setPosition(1);//servo stick is set to position 1
                telemetry.addData("b", "true");
                telemetry.update();
            }
            if (gamepad1.x) {//if game pad 1 a is being pressed...
                servoStickLeft2.setPosition(0);//servo stick is set to position 0
                telemetry.addData("Correct", "true");
                telemetry.update();
            }
            if (gamepad1.y) { //if game pad 1 b is pressed ...
                servoStickLeft2.setPosition(1);//servo stick is set to position 1
                telemetry.addData("Wrong", "true");
                telemetry.update();
            }
            idle();//waits to be caught up
        }
    }
    public void getJoyVals()
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
