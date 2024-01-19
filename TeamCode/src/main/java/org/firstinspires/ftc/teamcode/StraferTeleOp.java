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
        //Servo plane = hardwareMap.servo.get("plane");


        // Reverse one side of the motors
        // If it goes in reverse, reverse the other side.
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
       // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // output a message saying everything is initialized
        telemetry.addLine("Robot Internal Visualizer (RIV) -- Version 2.1.0\n\n");
        telemetry.addData("Robot Status", "◌ Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Set starting positions of wrist & claw
        wrist.setPosition(0.5);
        claw.setPosition(0.5);

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
                rightslide.setPower(.5);
            }
            else if (gamepad2.right_trigger > 0.1 && rightSlidePos >= 100) {
                rightslide.setPower(-.5);
            }
            else
                rightslide.setPower(0);

            if (gamepad2.dpad_up) {
                rightslide.setPower(1);
                leftslide.setPower(-1);
            }
            else if (gamepad2.dpad_down && rightSlidePos >= 100) {
                rightslide.setPower(-.75);
                leftslide.setPower(.75);
            }
            else if(gamepad2.dpad_down && rightSlidePos >= 100 && gamepad2.right_trigger > 0.1)
            {
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


            // NOTE: This will help with our next robot. Visual information is very useful!
            telemetry.addLine("Robot Internal Visualizer (RIV) -- Version 2.1.0\n\n");
            telemetry.addData("Robot Status", "✔ Running");
            telemetry.addData("➤ Active Motor(s)", "lf, lb, rf, rb, arm");
            telemetry.addData("➤ Active Servo(s)", "wrist, gripper, plane");
            telemetry.addData("➤ Current Runtime", getRuntime());
            // Front left and right are separated.
            telemetry.addLine("--------------Front--------------");
            telemetry.addLine("===> Left");
            telemetry.addData("Busy=", motorFrontLeft.isBusy());
            telemetry.addData("Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Direction", motorFrontLeft.getDirection());
            telemetry.addData("Power", motorFrontLeft.getPower());
            telemetry.addLine("===> Right");
            telemetry.addData("Busy=", motorFrontRight.isBusy());
            telemetry.addData("Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("Direction", motorFrontRight.getDirection());
            telemetry.addData("Power", motorFrontRight.getPower());
            // Back left and right are separated.
            telemetry.addLine("--------------Back--------------");
            telemetry.addLine("===> Left");
            telemetry.addData("Busy=", motorBackLeft.isBusy());
            telemetry.addData("Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("Direction", motorBackLeft.getDirection());
            telemetry.addData("Power", motorBackLeft.getPower());
            telemetry.addLine("===> Right");
            telemetry.addData("Busy=", motorBackRight.isBusy());
            telemetry.addData("Position", motorBackRight.getCurrentPosition());
            telemetry.addData("Direction", motorBackRight.getDirection());
            telemetry.addData("Power", motorBackRight.getPower());
            // Sliders left and right are separated
            telemetry.addLine("--------------Slider--------------");
            telemetry.addLine("===> Left");
            telemetry.addData("R Slide Pos: ", rightSlidePos);
            telemetry.addData("L Slide Pos: ", leftSlidePos);
            telemetry.update();
            // Arm, Wrist, and Gripper
            /*
            telemetry.addLine("--------------Arm--------------");
            telemetry.addData("Busy=", motorArm.isBusy());
            telemetry.addData("Position", motorArm.getCurrentPosition());
            telemetry.addData("Direction", motorArm.getDirection());
            telemetry.addData("Power", motorArm.getPower());
            telemetry.addData("▶ MotorMode", motorArm.getMode() + "\n");
            telemetry.addData("▶ MotorType", motorArm.getMotorType() + "\n");
            telemetry.addData("▶ Controller", motorArm.getController() + "\n");
            telemetry.addData("▶ PortNum", motorArm.getPortNumber() + "\n");
            telemetry.addData("▶ Device Name", motorArm.getDeviceName() + "\n");
            telemetry.addData("Is at ZeroPowerBehavior", motorArm.getZeroPowerBehavior());

            // Please note, Servos cannot use DCMotor/DCMotorSimple commands!
            // See Servo.java for list of Servo commands for telemetry
            telemetry.addLine("--------------Wrist--------------");
            telemetry.addData("Position", wrist.getPosition());
            telemetry.addData("Direction", wrist.getDirection());
            telemetry.addData("▶ Controller", wrist.getController() + "\n");
            telemetry.addData("▶ PortNum", wrist.getPortNumber() + "\n");
            telemetry.addData("▶ Device Name", wrist.getDeviceName() + "\n");
            telemetry.addLine("--------------Gripper--------------");
            telemetry.addData("Position", gripper.getPosition());
            telemetry.addData("Direction", gripper.getDirection());
            telemetry.addData("▶ Controller", gripper.getController() + "\n");
            telemetry.addData("▶ PortNum", gripper.getPortNumber() + "\n");
            telemetry.addData("▶ Device Name", gripper.getDeviceName() + "\n");

            */
            if (gamepad2.dpad_up) {
                //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor.setPower(0.2);
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setTargetPosition(armMotor.getCurrentPosition() + 100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (armMotor.getCurrentPosition() <= armMotor.getTargetPosition()) {
                armMotor.setPower(0.0);
            }

        }
    }
}
