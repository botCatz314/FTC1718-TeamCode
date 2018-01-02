package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ITSA-GAMINGHP2 on 11/2/2017.
 */
@TeleOp (name= "botCatzTeleOp", group = "TeleOp")
public class botCatzTeleOp extends LinearOpMode {
   private DcMotor leftMotor;
    private DcMotor lefttMotor2;
    private DcMotor rightMotor;
    private DcMotor rightMotor2;

    @Override
    public void runOpMode() throws InterruptedException
    {

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        lefttMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");

        waitForStart();

        while(opModeIsActive())
        {
            leftMotor.setPower(-gamepad1.left_stick_y);
            lefttMotor2.setPower(-gamepad1.left_stick_y);
            rightMotor.setPower(-gamepad1.right_stick_y);
            rightMotor2.setPower(-gamepad1.right_stick_y);

            idle();
        }

        }
    }

