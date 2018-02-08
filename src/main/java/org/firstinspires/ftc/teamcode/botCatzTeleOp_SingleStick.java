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
 * tele op code for robot
 * botCatzTeleOp_SingleStick
 * v3.0.2
 * 2/5/2018
 */
@TeleOp
public class botCatzTeleOp_SingleStick extends LinearOpMode {

    private float x, y, z, w, pwr;
    private static double deadzone = 0.2;
    private static boolean TankDriveActive = false, clawHold = false;
    private static boolean lsExtendPos = false, lsExtendNeg = false, lsRetractPos = false, lsRetractNeg = false;

    // declare motors as variables
    private DcMotor leftMotor, rightMotor; //declares drive motors
    private DcMotor slideMotor, armHeight, slideReverse; //declares attachment motors

    //declares motors as variables
    private Servo servoStickRight1,servoStickLeft2; //declares servos with regular range of motion
    private DcMotor clawMotor;

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
        clawMotor = hardwareMap.dcMotor.get("clawMotor");
        servoStickRight1 = hardwareMap.servo.get("servoStickRight1");
        servoStickLeft2 = hardwareMap.servo.get("servoStickLeft2");

        //sets parameters
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        slideReverse.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();//waits until we hit play

        while(opModeIsActive()) {//it loops the code inside until we hit stop

            if (TankDriveActive == true){
                /*leftMotor.setPower(gamepad1.left_stick_y);//sets power of leftMotor2 = to joystick value
                rightMotor.setPower(gamepad1.right_stick_y);//sets power of rightMotor2 = to joystick value
                leftMotor.setPower(gamepad1.right_stick_x);
                rightMotor.setPower(-gamepad1.right_stick_x);
                */
                //leftMotor.setPower(-gamepad1.left_stick_y);
                leftMotor.setPower(gamepad1.left_stick_y);
                //rightMotor.setPower(-gamepad1.right_stick_y);
                rightMotor.setPower(gamepad1.right_stick_y);
            }
            else {
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
            }
            if (gamepad1.a) {
                TankDriveActive = true;
            }
            if (gamepad1.b) {
                TankDriveActive = false;
            }

            //controls the linearSlide

            if(gamepad2.right_stick_y > 0.1){
                slideReverse.setPower(0.5*gamepad2.right_stick_y);
                slideMotor.setPower(gamepad2.right_stick_y);
            } else if(gamepad2.right_stick_y < -0.1){
                slideReverse.setPower(0.5*gamepad2.right_stick_y);
                slideMotor.setPower(gamepad2.right_stick_y);
            } else {
                slideReverse.setPower(0);
                slideMotor.setPower(0);
            }

            if (gamepad2.dpad_down){
                lsExtendNeg = true;
                if (lsExtendNeg) lsExtendPos = false;
            }
            if (gamepad2.dpad_up){
                lsExtendPos = true;
                if (lsExtendPos) lsExtendNeg = false;
            }
            if (gamepad2.dpad_left){
                lsRetractPos = true;
                if (lsRetractPos) lsRetractNeg = false;
            }
            if (gamepad2.dpad_right){
                lsRetractNeg = true;
                if (lsRetractNeg) lsRetractPos = false;
            }

            if (lsExtendNeg) slideMotor.setPower(-0.25);

            if (lsExtendPos) slideMotor.setPower(0.25);

            if (lsRetractNeg) slideReverse.setPower(-0.25);

            if (lsRetractPos) slideReverse.setPower(0.25);
            
            //sets power of clawMotor
            clawMotor.setPower(-gamepad2.right_trigger); //Close claw
            clawMotor.setPower(gamepad2.left_trigger); //Open claw

            // clawMotor hold
            if (gamepad2.right_bumper){
                clawHold = true;
            }
            if (gamepad2.left_bumper){
                clawHold = false;
            }
            if (clawHold){
                // hold claw closed
                clawMotor.setPower(-0.4);
            }

            //controls the height of the linear slide
            armHeight.setPower(-gamepad2.left_stick_y);//sets the motor controlling arm height equal to the left stick

            //gives drivers control of the servoStick in case it does not pull up
            if (gamepad2.a) {
                servoStickRight1.setPosition(1);
                telemetry.addData("a", "true");
                telemetry.update();
            }
            if (gamepad2.b) {
                servoStickRight1.setPosition(0);
                telemetry.addData("b", "true");
                telemetry.update();
            }
            if (gamepad2.x) {
                servoStickLeft2.setPosition(1);
                telemetry.addData("Correct", "true");
                telemetry.update();
            }
            if (gamepad2.y) {
                servoStickLeft2.setPosition(0);
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
