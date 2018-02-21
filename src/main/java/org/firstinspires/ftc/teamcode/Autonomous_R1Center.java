package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */

@Autonomous(name = "Autonomous_R1_Center_V3.14", group = "Pushbot" )

public class Autonomous_R1Center extends LinearOpMode {
    private DcMotor leftMotor, rightMotor,clawMotor,armHeight; //Declares the drive motors

    private Servo servoStickLeft2, servoStickRight1, blockFlicker; //declares servos

    private ColorSensor colorSensorLeft, colorSensorRight; //declares color sensors

    BNO055IMU imu; //declares integrated gyro
    Orientation lastAngle = new Orientation();

    private double Kp = 0.35, error, globalAngles, powerOff = 0;
    private double pi = 3.1415926535897932;
    double threshold = .25, colorThreshold;


    public boolean Right = false; //Variable for the Vuforia code
    public boolean Center = false;//Variable for the Vuforia code
    public boolean Left = false;  //Variable for the Vuforia code
    public static final String TAG = "Vuforia VuMark Sample"; //The Sample used in Vuforia
    public boolean UsingEncoders = false;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

 
    // Runs at init time
    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //Uses camera viewer on the phone
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //Enables camera viewer on the phone
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parametersV.vuforiaLicenseKey = "AeOpxj3/////AAAAGa1hky4Ahkp6jA7uCGunP+KJAZb3Di06YSh1ToEAxDmlWGeqxY3Mp26DqFw1P5Lyc/gFq992XUJ2bf8QtwYWln76jzRISvwAoSotdCOMreIL6fpbK4fdsAG9u85FTJlPDsOMY5u9YktxQ/JERWyrQC/NhAxJX+RDVtTouFnrUx/EI8CJDHR/IFcHnQ4KIJdCfQBoeC6+qMJ1RCa2lo2BFPcQv4blFatYz4Z0P+0XVhiza0t0mwJXKzTlwq+c4V9X0nWseTQZXnmgbB0kwQx+m/pGzr9ImML9WhSiWp5qPjyqDYitWs7cU/zWLFFT1wWpW7KkhQ+boQ2zwUsYKemRKY21LV9lkHh5/2a7bJWqKHY/"; //Key for using Vuforia in code
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Decides which camera to use
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersV); //Creates Vuforia Localizer
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark"); //Decides which pictures to use
        VuforiaTrackable relicTemplate = relicTrackables.get(0); //Base pictures
        relicTemplate.setName("relicVuMarkTemplate"); // Can help in debugging; otherwise not necessary


        //declares drive motor
        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //gets properties of left motor from phone
        rightMotor = hardwareMap.dcMotor.get("rightMotor"); //gets properties for second left motor from phone
        clawMotor = hardwareMap.dcMotor.get("clawMotor");
        armHeight = hardwareMap.dcMotor.get("armHeight");

        double down = 1;
        double up = 0;

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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        imu.readCalibrationData(); //calibrates gyro
        imu.isGyroCalibrated(); //checks that gyro is calibrated
        //shows user that gyro is calibrated


        //stops and resets encoders of Drive Motors and runs using encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicTrackables.activate(); //Activates Vuforia
        
        // Everything above is the init.
        // This blocks and everything below is after start
        waitForStart(); //waits until the user presses play
        
        // opModeIsActive will return false when the user hits stop.
        while (opModeIsActive()) {
            // VuMark == camera access

            //Sets variable for figuring out which picture it is
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //Reference to picture
            //Checks if the VuMark is unknown and acts based on that
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //Events for if VuMark is unknown
                // Do this when we know what the cypher is
                telemetry.addData("VuMark", "%s visible", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); //Gets the position of the picture
                //Turns it into rotation and position coordinates

                // Extract the position of the relic if there is one.
                if (pose != null) { //Events to track position of the picture relative to the robot.
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

            // At this point we might have a known relic position.
            // ??? if the relic is unknown.

            //Runs a check to see which of the picture is active
            Center = (vuMark == RelicRecoveryVuMark.CENTER);
            Right = (vuMark == RelicRecoveryVuMark.RIGHT);
            Left = (vuMark == RelicRecoveryVuMark.LEFT);
            if (Center) {
                log("Center");
            } else if (Right) {
                log("Right");
            } else if (Left) {
                log("Left");
            } else {
                log("VuMark", "not visible");
            }

            // Write out the debug info
            telemetry.update();

            servoStickRight1.setPosition(down);
            sleep(2000);
            if (DriveFunctions.ReadColor(colorSensorRight) == 1){
                log("Color is Blue");
                DriveFunctions.Turn(0.3,-0.3,leftMotor,rightMotor);
                sleep(200);
                DriveFunctions.Brake(leftMotor,rightMotor);
                servoStickRight1.setPosition(up);
                sleep(2000);
                DriveFunctions.Turn(-0.3,0.3,leftMotor,rightMotor);
                sleep(200);
                DriveFunctions.Brake(leftMotor,rightMotor);
            }
            else if (DriveFunctions.ReadColor(colorSensorRight) == 0){
                log("Color is Red");
                DriveFunctions.Turn(-0.3,0.3,leftMotor,rightMotor);
                sleep(200);
                DriveFunctions.Brake(leftMotor,rightMotor);
                servoStickRight1.setPosition(up);
                sleep(2000);
                DriveFunctions.Turn(0.3,-0.3,leftMotor,rightMotor);
                sleep(200);
                DriveFunctions.Brake(leftMotor,rightMotor);
            }
            else {
                log("Color is not visible");
            }

            log("Begin off-board driving");
            servoStickRight1.setPosition(up);
            sleep(2000);

            log("Ease off the board");
            DriveFunctions.Turn(0.3,0.7, leftMotor,rightMotor);
            sleep(1000);
            DriveFunctions.Brake(leftMotor,rightMotor);

            sleep(1000);

            log("Easy left Turn();();");
            DriveFunctions.Turn(-0.3,0.7,leftMotor,rightMotor);
            sleep(200);
            DriveFunctions.Brake(leftMotor,rightMotor);

            sleep(1000);

            log("Straight ish");
            DriveFunctions.Turn(0.3,0.7,leftMotor,rightMotor);
            sleep(1100);
            DriveFunctions.Brake(leftMotor,rightMotor);

            sleep(1000);

            //what if we're at the wall?
            log("Backup in case we're at the wall");
            DriveFunctions.BackUp(leftMotor, rightMotor, 0.3);
            sleep(200);
            DriveFunctions.Brake(leftMotor,rightMotor);

            log("Flick");
            blockFlicker.setPosition(0);
            sleep(1000);
            blockFlicker.setPosition(1);
            sleep(1000);
            blockFlicker.setPosition(0);
            sleep(1000);
            blockFlicker.setPosition(1);
            sleep(1000);


            log("Straight");
            DriveFunctions.Turn(0.3,0.3,leftMotor,rightMotor);
            sleep(500);
            DriveFunctions.Brake(leftMotor,rightMotor);

            //final back up
            log("Backup one last time");
            DriveFunctions.BackUp(leftMotor, rightMotor, 0.3);
            sleep(300);
            DriveFunctions.Brake(leftMotor,rightMotor);

            if (Center) {
                log("Center");
            } else if (Right) {
                log("Right");
            } else if (Left) {
                log("Left");
            } else {
                log("VuMark", "not visible");
            }

            sleep(300000);

        }

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

    //declares function that calculates math to drive by encoder
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
    public double CalculateError(double desiredAngle) {
        double error; //sets a variable to hold the error
        error = desiredAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle; //subtracts the input angle from the imu reading
        telemetry.addData("error", error); //sets the telemetry to send error to phone
        telemetry.update(); //pushes previously set data to phone
        return error; //sends back error value to be used
    }
    private void turnGyro(double angle, double speed){
        telemetry.addData("turn Gyro", angle); //sets telemetry equal to the input angle
        telemetry.update();//pushes telemetry to phone
        while(opModeIsActive() && !OnHeading(speed, angle)){ //while the robot is not Onheading (see onHeading function)
            telemetry.addData("turning", "check"); //sets telemetry
            telemetry.update(); //pushes telemetry
        }
    }
    public boolean OnHeading(double speed, double angle) {
        double error, steer, leftSpeed, rightSpeed; //sets our doubles
        boolean onTarget = false; //sets onTarget to false
        error = CalculateError(angle); //gets the error from the calculate error function
        if(error <= threshold ){ //if the error is not less than a double we declare
            steer = 0.0; //sets steer double to 0
            leftSpeed= 0.0; //sets motor speeds to 0
            rightSpeed = 0.0;
            onTarget = true; //sets onTarget to true
        }
        else {
            steer = adjustHeading(error); //sets steer = adjust heading with the error as input (see adjustHeading function)
            rightSpeed = -(speed * steer); // sets right speed to input speed multiplied by the adjusted heading
            leftSpeed = -rightSpeed; //sets left speed to the opposite of the right speed
        }
        rightMotor.setPower(rightSpeed); //sets motor power to the speed
        leftMotor.setPower(leftSpeed); //sets motor power to the speed
        return onTarget; // the while statement in turnGyro will stop if this value is true
    }
    private double adjustHeading(double error){
        double  Kp = .15, Ki = 0, Kd = 0; //declares PID constants
        double errorPrior = 0; //declares prior error
        double integral = 0, derivative = 0; //sets the original integral and derivative values to 0
        ElapsedTime turning = new ElapsedTime(); //sets a new elapsed time for use to use
        turning.reset(); //sets the elapsed time over
        while(true) { //it will forever calculate
            integral = integral + (error * turning.time()); //integral value = previous integral value (initial being 0) + the error * elapsed time
            derivative = (error - errorPrior)/turning.time(); //derivative value = error subtracted by previous error divided by the elapsed time
            errorPrior = error; //error prior = error which will be updated soon
            return Range.clip((error * Kp)+(Ki*integral)+(Kd*derivative), -1, 1); //return P+I+D between -1 and 1 to be multiplied by the speed
        }
    }
}
