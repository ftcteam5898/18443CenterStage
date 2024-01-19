package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This particular OpMode executes a POV Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back and strafes left & right.
 * The Right stick rotates the robot left and right.
 *
 */

@TeleOp(name="Strafer-new controls", group="Tele")
public class StraferTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("lf");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("lb");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rf");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rb");

        DcMotor leftslide = hardwareMap.dcMotor.get("lslide");
        DcMotor rightslide = hardwareMap.dcMotor.get("rslide");
        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor leftIntake = hardwareMap.dcMotor.get("lintake");
        DcMotor rightIntake = hardwareMap.dcMotor.get("rintake");

        //Declare servos
        Servo leftBumper = hardwareMap.servo.get("lbumper");
        Servo rightBumper = hardwareMap.servo.get("rbumper");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo claw = hardwareMap.servo.get("claw");
        Servo plane = hardwareMap.servo.get("plane");


        // Reverse one side of the motors
        // If it goes in reverse, reverse the other side.
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // output a message saying everything is initialized
        telemetry.addLine("Robot Internal Visualizer (RIV) -- Version 2.1.0\n\n");
        telemetry.addData("Robot Status", "◌ Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        plane.setPosition(0);
        //Set starting positions of wrist & claw
        wrist.setPosition(0.5);
        claw.setPosition(0);

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            /////// Driver 1 ////////////

            // Driving controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            // Bumpers
            if (gamepad1.left_bumper) {
                leftBumper.setPosition(0);
            }
            else {
                leftBumper.setPosition(1);
            }
            if (gamepad1.right_bumper) {
                rightBumper.setPosition(1);
            }
            else {
                rightBumper.setPosition(0);
            }

            // Intake
            if (gamepad1.dpad_up) {
                leftIntake.setPower(1);
                rightIntake.setPower(1);
            }
            else if (gamepad1.dpad_down) {
                leftIntake.setPower(-1);
                rightIntake.setPower(-1);
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }


            ///////// Driver 2 //////////////
            int rightSlidePos = rightslide.getCurrentPosition();
            int leftSlidePos = leftslide.getCurrentPosition();

            if (gamepad2.right_bumper) {
                leftslide.setPower(-.5);
            }
            else if (gamepad2.right_trigger > 0.1 && leftSlidePos <= -30) {
                leftslide.setPower(.5);
            }
            else if (gamepad2.dpad_up) {
                rightslide.setPower(1);
                leftslide.setPower(-1);
            }
            else if(gamepad2.left_bumper && gamepad2.dpad_down)
            {
                rightslide.setPower(-.75);
            }
            else if (!gamepad2.left_bumper && gamepad2.dpad_down && rightSlidePos >= 30) {
                rightslide.setPower(-.75);
                leftslide.setPower(.75);
            }

            else {
                rightslide.setPower(0);
                leftslide.setPower(0);
            }

            if (gamepad2.x) {
                claw.setPosition(.5); //open
            }
            else if (gamepad2.b) {
                claw.setPosition(0); //close
            }

            if (gamepad2.y) {
                wrist.setPosition(.5); // intake
            }
            else if (gamepad2.a) {
                wrist.setPosition(1); //backboard
            }
            if (gamepad2.dpad_left && runtime.seconds()>= 90)
            {
                plane.setPosition(0.5);
            }
            if (gamepad2.guide || gamepad2.ps)
            {
                plane.setPosition(0.5);
            }



            // NOTE: This will help with our next robot. Visual information is very useful!
            telemetry.addData("Gamepad 2 Button pressed: ", gamepad2);
            telemetry.addData("elapsed time: ", runtime.seconds());
            telemetry.update();

        }
    }
}
