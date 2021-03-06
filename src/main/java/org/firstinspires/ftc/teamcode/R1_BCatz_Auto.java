/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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


@Autonomous(name="R1_BCatz_Auto", group ="Pushbot")
@Disabled
public class R1_BCatz_Auto extends LinearOpMode {

    private DcMotor leftMotor, rightMotor; //Declares the drive motors

    private Servo servoStickLeft2, servoStickRight1, blockFlicker; //declares servos

    private ColorSensor colorSensorLeft, colorSensorRight; //declares color sensors

    BNO055IMU imu; //declares integrated gyro
    Orientation lastAngle = new Orientation();

    private double Kp = 0.35, error, globalAngles, powerOff = 0;
    private double pi = 3.1415926535897932;

    public boolean Right = false; //Variable for the Vuforia code
    public boolean Center = false;//Variable for the Vuforia code
    public boolean Left = false;  //Variable for the Vuforia code
    public static final String TAG = "Vuforia VuMark Sample";
    public boolean UsingEncoders = false;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;



    @Override public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //Uses camera viewer on the phone
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parametersV.vuforiaLicenseKey = "AeOpxj3/////AAAAGa1hky4Ahkp6jA7uCGunP+KJAZb3Di06YSh1ToEAxDmlWGeqxY3Mp26DqFw1P5Lyc/gFq992XUJ2bf8QtwYWln76jzRISvwAoSotdCOMreIL6fpbK4fdsAG9u85FTJlPDsOMY5u9YktxQ/JERWyrQC/NhAxJX+RDVtTouFnrUx/EI8CJDHR/IFcHnQ4KIJdCfQBoeC6+qMJ1RCa2lo2BFPcQv4blFatYz4Z0P+0XVhiza0t0mwJXKzTlwq+c4V9X0nWseTQZXnmgbB0kwQx+m/pGzr9ImML9WhSiWp5qPjyqDYitWs7cU/zWLFFT1wWpW7KkhQ+boQ2zwUsYKemRKY21LV9lkHh5/2a7bJWqKHY/";
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Decides which camera to use
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersV);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark"); //Decides which pictures to use
        VuforiaTrackable relicTemplate = relicTrackables.get(0); //Base pictures
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone

        //declares sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets properties of gyro from phone
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

        //sets parameters of gyro
        BNO055IMU.Parameters Parameters = new BNO055IMU.Parameters();
        Parameters.mode = BNO055IMU.SensorMode.IMU;
        Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Parameters.loggingEnabled = false;
        imu.initialize(Parameters);

        imu.readCalibrationData(); //calibrates gyro
        imu.isGyroCalibrated(); //checks that gyro is calibrated
        //shows user that gyro is calibrated

        servoStickRight1.setPosition(1);//drop servo stick
        servoStickRight1.setPosition(0);//raise servo stick
        telemetry.addData("You messed up already,", "Wes");


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        relicTrackables.activate(); //Activates Vuforia
        waitForStart();

        while (opModeIsActive()) {
                //Sets variable for figuring out which picture it is
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                //Checks if the VuMark is unknown and acts based on that
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); //Gets the angle of the picture
                    //Turns it into rotation and position coordinates
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);
                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                }
                //Runs a check to see which of the picture is active
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    Center = true;
                    telemetry.addData("Bool", "Center");
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    Right = true;
                    telemetry.addData("Bool", "Right");
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    Left = true;
                    telemetry.addData("Bool", "Left");
                }
                if (vuMark != RelicRecoveryVuMark.CENTER) {
                    Center = false;
                }
                if (vuMark != RelicRecoveryVuMark.RIGHT) {
                    Right = false;
                }
                if (vuMark != RelicRecoveryVuMark.LEFT) {
                    Left = false;
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
                telemetry.update();

            telemetry.update();

            //sleep(300);
            //DriveWithEncoders(20,.3);
            //sleep(200);

            //rightMotor.setPower(.3);
            //leftMotor.setPower(-0.3);
            //sleep(600);
            //rightMotor.setPower(0);
            //leftMotor.setPower(0);

            sleep(30000);


        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private void DriveWithEncoders(double distance, double speed) {
        //computes mathematical formulas that translate between encoder counts and real distance
        double off = 0;
        double encoderCounts = 1120;
        double driveGearReduction = 4.0;
        double wheelDiameter = 9;
        double countsPerMM = (encoderCounts * driveGearReduction) / (wheelDiameter * pi);

        //declares two variables to serve as our left and right motor
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            //adds desired distance to the current reading of encoders to ensure accurate measurements
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (countsPerMM * distance);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (countsPerMM * distance);

            //sets the motors' target positions
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            //sets the motors' mode
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //turn the motors on for input speed
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            //loops and gives telemetry until the motors are finished
            while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
                telemetry.addData("target left position: ", newLeftTarget);
                telemetry.addData("target right position: ", newRightTarget);
                telemetry.addData("current left position", leftMotor.getCurrentPosition());
                telemetry.addData("current right position: ", rightMotor.getCurrentPosition());
                telemetry.addData("rightMotor: ", rightMotor.isBusy());
                telemetry.addData("leftMotor", leftMotor.isBusy());
                telemetry.addData("rightSpeed", rightMotor.getPower());
                telemetry.addData("leftSpeed", leftMotor.getPower());
                telemetry.update();
            }

            //turns power of motors off
            leftMotor.setPower(off);
            rightMotor.setPower(off);
        }
    }

    public double readGyro(){
        //gets value of Gyro
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES);
        double deltaAngle = angle.firstAngle -lastAngle.firstAngle; //change in angle = new - old

        if(deltaAngle < 180){
            deltaAngle +=360; //keeps delta angle within valid range
        }
        else if(deltaAngle > 180){
            deltaAngle -= 360; //keeps delta angle within valid range
        }
        globalAngles +=deltaAngle; //global Angle = globalAngle + deltaAngle
        lastAngle = angle; //sets last angle to the angle measurement we just received
        return globalAngles;
    }

    public double CalculateError(double desiredAngle){
        double error;
        error = desiredAngle - readGyro();
        return error;
    }

    public boolean OnHeading(double speed, double angle, double Kp){
        double error, steer, leftSpeed, rightSpeed;
        boolean onTarget = false;
        error = CalculateError(angle);

        return onTarget;
    }
}