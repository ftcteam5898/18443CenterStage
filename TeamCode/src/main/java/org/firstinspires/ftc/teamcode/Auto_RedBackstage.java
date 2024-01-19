package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="Auto_RedBackstage", group="Red Auto", preselectTeleOp="Strafer Tele Op")
public class Auto_RedBackstage extends LinearOpMode{
    // variable declaration & setup
    DcMotor frontleft, frontright, backleft, backright, leftSlide;

    Servo leftBumper;
    Servo rightBumper;
    Servo wrist;
    Servo claw;

    // Set up webcam, processor, & vision portal
    /*private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "18443_red_centerstage.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {"redCastle"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;*/

    // motor counts per rotation (ticks/pulses per rotation)
    // check motor specs from manufacturer
    // 537.7 is for GoBilda 312 RPM Yellow Jacket motor
    double cpr = 537.7;

    // adjust gearRatio if you have geared up or down your motors
    double gearRatio = 1;

    // wheel diameter in inches
    // 3.779 is for the GoBilda mecanum wheels
    double diameter = 3.779;

    //counts per inch: cpr * gear ratio / (pi * diameter (in inches))
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);

    // use calibrate auto to check this number before proceeding
    double bias = 0.94; // adjust based on calibration opMode
    
    double strafeBias = 0.9;//change to adjust only strafing movement
    //
    double conversion = cpi * bias;
    //
    IMU imu;

    @Override
    public void runOpMode(){

        initGyro();
        //initTfod();

        // setup motors
        frontleft = hardwareMap.dcMotor.get("lf");
        frontright = hardwareMap.dcMotor.get("rf");
        backleft = hardwareMap.dcMotor.get("lb");
        backright = hardwareMap.dcMotor.get("rb");

        leftSlide = hardwareMap.dcMotor.get("lslide");
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBumper = hardwareMap.servo.get("rbumper");
        leftBumper = hardwareMap.servo.get("lbumper");
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");


        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);



        leftBumper.setPosition(0.1);
        rightBumper.setPosition(1);
        closeClaw();
        sleep(1000);
        wrist.setPosition(.4); // lift a little off the ground

        /*while(tfod.getRecognitions().size() == 0)
        {
            tfod.getRecognitions();
        }
        telemetryTfod();*/


        // wait for Start to be pressed
        waitForStart();

        // Call functions here
        int pos = 2;

        sleep(500);
        if (pos == 1)
        {
            telemetry.addLine("Object location: Left");
            telemetry.update();

            // go forward, rotate, and back up to drop off the purple pixel on the tape line
            forward(22, .5);
            sleep(1000);
            turnLeft(90, .25);
            sleep(1000);
            forward(8, .5);
            sleep(1000);
            back(2, 0.5);

            /*// back up to the board
            back(34, 1);

            // drop off yellow pixel
            extendSlide(2);
            dumpPixel();
            closeSlide(2);

            // strafe right and park
            strafeLeft(30, .5);
            back(12, 1);*/
        }
        else if (pos == 3)
        {
            telemetry.addLine("Object location: Right");
            telemetry.update();

            // strafe right, go forward,  and back up to drop off the purple pixel on the tape line
            strafeRight(8, .5);
            forward(22, .5);
            back(2, 0.5);

            // turn left and back up to the board
            turnLeft(85, 0.5);
            back(24, .5);

            // drop off yellow pixel
            extendSlide(2);
            dumpPixel();
            closeSlide(2);

            // strafe right and park
            strafeLeft(30, .5);
            back(12, .5);
        }
        else
        {
            telemetry.addLine("Object location: Middle");
            telemetry.update();

            // go forward and back up to drop off the purple pixel on the tape line
            forward(28, .5);
            back(8, 0.5);

            // turn left and back up to the board
            turnLeft(85, 0.25);
            back(32, .5);

            // drop off yellow pixel
            extendSlide(2);
            dumpPixel();
            closeSlide(2);

            // strafe right and park
            strafeLeft(30, .5);
            back(12, .5);
        }



    }







    /**
     * Use to make the robot go forward a number of inches
     * @param inches distance to travel in inches
     * @param speed has a range of [0,1]
     */
    public void forward(double inches, double speed){ moveToPosition(inches, speed); }

    /**
     * Use to make the robot go backward a number of inches
     * @param inches distance to travel in inches
     * @param speed has a range of [0,1]
     */
    public void back(double inches, double speed){ moveToPosition(-inches, speed); }

    /**
    Rotate the robot left
    @param degrees the amount of degrees to rotate
    @param speed has a range of [0,1]
     */
    public void turnLeft(double degrees, double speed){ turnWithGyro(degrees, -speed); }

    /**
    Rotate the robot right
    @param degrees the amount of degrees to rotate
    @param speed has a range of [0,1]
     */
    public void turnRight(double degrees, double speed){ turnWithGyro(degrees, speed); }

    /**
    Strafe left
    @param inches the distance in inches to strafe
    @param speed has a range of [0,1]
     */
    public void strafeLeft(double inches, double speed){ strafeToPosition(-inches, speed); }

    /**
    Strafe right
    @param inches the distance in inches to strafe
    @param speed has a range of [0,1]
     */
    public void strafeRight(double inches, double speed){ strafeToPosition(inches, speed); }


    /**
     * Opens the gripper on the arm of the robot
    */
    public void openClaw()
    {
        claw.setPosition(0.5);
    }

    /**
     * Opens the gripper on the arm of the robot
     *
     */
    public void closeClaw()
    {
        claw.setPosition(0);
    }

    /**
     * Releases the yellow pixel onto the backdrop
     *
     */
    public void dumpPixel()
    {
        wrist.setPosition(1);
        sleep(500);
        openClaw();
        sleep(500);
        closeClaw();
        sleep(500);
        wrist.setPosition(.5);
        sleep(500);
    }

    /**
     * Extends the claw slide
     * @param time number of seconds that slide should extend
     */
    public void extendSlide(int time)
    {
        leftSlide.setPower(-.5);
        sleep(time * 1000);
    }

    /**
     * Closes the claw slide
     * @param time number of seconds that slide should extend
     */
    public void closeSlide(int time)
    {
        leftSlide.setPower(.5);
        sleep(time * 1000);
    }

    /**
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        int move = (int)(Math.round(inches*conversion));
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    /**
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 2).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = -orientation.getYaw(AngleUnit.DEGREES);//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //

        double first;
        double second;

        //
        if (speedDirection > 0){//set target positions

            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }

        }else{

            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //

        }
        //

        double firsta = convertify(first - 2);//178
        double firstb = convertify(first + 2);//-178
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        double seconda = convertify(second - 2);//178
        double secondb = convertify(second + 2);//-178
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }

        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        int move = (int)(Math.round(inches * cpi * strafeBias));
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);

        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            telemetry.addData("Working...", " ");
            telemetry.update();}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    /**
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 360){
            degrees = degrees - 360;
        }
        else if(degrees < -180){
            degrees = 360 + degrees;
        }
        else if(degrees > 179){
            degrees = -(360 - degrees);
        }
        return degrees;
    }

    /**
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        // Check the orientation of the Rev Hub
        // more info on ftc-docs.firstinspires.org
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
/*    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
*/
    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
/*    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for(int i = 0; i < 50; i++)
            currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.update();
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (x < 200)
                telemetry.addLine("Left position");
            else if (x > 1000)
                telemetry.addLine("Right position");
            else
                telemetry.addLine("Middle position");
            telemetry.update();

        }   // end for() loop

    }   // end method telemetryTfod()
*/
    /**
     *
     */
/*    private int objRecog() {
        while(tfod.getRecognitions().size() == 0)
        {
            tfod.getRecognitions();
            telemetry.addLine("Getting recognitions");
            telemetry.update();
        }
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (recognition.getConfidence()*100 > 65)
            {
                if (x < 200) return 1;
                else if (x > 1000) return 3;
                else return 2;
            }
        }   // end for() loop
        return 2;
    }   // end method objRecog()

*/
}
