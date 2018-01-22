package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.code.Attribute;
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


/**
 * Created by ITSA-GAMINGHP2 on 11/9/2017.
 */

@Autonomous(name = "Autonomous_B1", group = "Concept" )
@Disabled
public class Autonomous_B1_Left extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2; //Declares the motors
    private ColorSensor colorSensor;     //declares the color sensor
    private Servo servoStick; //declares servos
    double powerOff = 0; //creates a variable equal to zero so that the motors can turn off without the use of a magic number
    BNO055IMU imu; //declares integrated gyro
    private boolean NEEDSTOBEDELETED = true;
    public boolean Right = false;
    public boolean Center = false;
    public boolean Left = false;
    public static final String TAG = "Vuforia VuMark Sample";
    public boolean UsingEncoders = false;

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

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
                /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parametersV.vuforiaLicenseKey = "AeOpxj3/////AAAAGa1hky4Ahkp6jA7uCGunP+KJAZb3Di06YSh1ToEAxDmlWGeqxY3Mp26DqFw1P5Lyc/gFq992XUJ2bf8QtwYWln76jzRISvwAoSotdCOMreIL6fpbK4fdsAG9u85FTJlPDsOMY5u9YktxQ/JERWyrQC/NhAxJX+RDVtTouFnrUx/EI8CJDHR/IFcHnQ4KIJdCfQBoeC6+qMJ1RCa2lo2BFPcQv4blFatYz4Z0P+0XVhiza0t0mwJXKzTlwq+c4V9X0nWseTQZXnmgbB0kwQx+m/pGzr9ImML9WhSiWp5qPjyqDYitWs7cU/zWLFFT1wWpW7KkhQ+boQ2zwUsYKemRKY21LV9lkHh5/2a7bJWqKHY/";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersV);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

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

        relicTrackables.activate();
        waitForStart(); //waits until the user presses play
        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
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
            if (vuMark == RelicRecoveryVuMark.CENTER){
                Center = true;
                telemetry.addData("Bool", "Center");
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT){
                Right = true;
                telemetry.addData("Bool", "Right");
            }
            if (vuMark == RelicRecoveryVuMark.LEFT){
                Left = true;
                telemetry.addData("Bool", "Left");
            }
            if (vuMark != RelicRecoveryVuMark.CENTER){
                Center = false;
            }
            if (vuMark != RelicRecoveryVuMark.RIGHT){
                Right = false;
            }
            if (vuMark != RelicRecoveryVuMark.LEFT){
                Left = false;
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        servoStick.setPosition(1);//servo stick down motion
        sleep(3000);//pause code for 2 seconds
       //
            try {
                if (colorSensor.red() > colorSensor.blue()) {// asking if red's presence is greater than blue
                    telemetry.addData("red: ", colorSensor.red());//prints red's value to phone
                    telemetry.addData("blue: ", colorSensor.blue());//prints blue's value to phone
                    telemetry.update();//prints the telemetry to screen
                    sleep(4000); //pause for 4 seconds
                //    Drive_Forwards(.2);// moves forwards at 40% power
                    sleep(500); // pause for 1/2 second
                //    Brake(); // stops wheel motion
                    sleep(500);
                    servoStick.setPosition(0); // bring up the jewel stick
                 //   Drive_Backwards(.2);//drives to roughly the same place as other part of if statement
                    sleep(1000);//pauses for one second
                //    Brake();//stops all wheel movement
                } else if (colorSensor.blue() > colorSensor.red()) {// if blue is greater than red...
                    telemetry.addData("red: ", colorSensor.red());// queues value of red to be printed
                    telemetry.addData("blue: ", colorSensor.blue());//queues value of blue to be printed
                    telemetry.update();// prints queued data to phone
                    sleep(4000); // stops for 4 seconds
                //    Drive_Backwards(.2); // drives backwards at 40% power
                    sleep(500); // stops for 1/2 a seconds
                 //   Brake();// stops wheel movement
                    servoStick.setPosition(0);// bring up jewel whacker
                }
            }
            catch(Error e) {

                telemetry.addData("we messed up...", "aww");
                sleep(1000);
            }
        servoStick.setPosition(0);// auxiliary bringing up of jewel whacker6
     /*   if (Left = true && UsingEncoders = false) {

            Drive_Backwards(.2);
            sleep(600);
            Brake();

            Drive_Backwards(.3);
            sleep(2300);
            Brake();

            Turn_Right(.4);
            sleep(600);
            Brake();

            telemetry.update();
            sleep(15000);
        }
        if (Center = true && UsingEncoders = false) {

            Drive_Backwards(.2);
            sleep(600);
            Brake();

            Turn_Right(.4);
            sleep(600);
            Brake();

            Drive_Backwards(.3);
            sleep(2300);
            Brake();

            telemetry.update();
            sleep(15000);
        }
       // if (Right = true && UsingEncoders = false) {

            Drive_Backwards(.2);
            sleep(600);
            Brake();

            Turn_Right(.6);
            sleep(600);
            Brake();

            Drive_Backwards(.3);
            sleep(2300);
            Brake();

            telemetry.update();
            sleep(15000);
        }

    }
    private void Drive_Forwards(double power){//function for driving forwards
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
    */}
}
