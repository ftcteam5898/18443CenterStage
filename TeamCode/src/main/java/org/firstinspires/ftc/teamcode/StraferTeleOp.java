package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This particular OpMode executes a POV Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back and strafes left & right.
 * The Right stick rotates the robot left and right.
 *
 */

@TeleOp(name="Strafer Tele Op", group="Starter Code")
public class StraferTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("lf");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("lb");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rf");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rb");
        DcMotor motorArm = hardwareMap.dcMotor.get("arm");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo gripper = hardwareMap.servo.get("gripper");
        Servo plane = hardwareMap.servo.get("plane");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double wristCurrentPos = 0.5;
        wrist.setPosition(wristCurrentPos);
        plane.setPosition(0.0);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

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
            double wristUpPos = 1.0;
            double wristDownPos = 0.0;
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

        // ARM & WRIST
            if (gamepad2.dpad_up)
            {
                motorArm.setPower(1);
            } else if (gamepad2.dpad_down) {
                motorArm.setPower(-1);
            }
            else motorArm.setPower(0);  // Arm remains in its static position.

            if (gamepad2.y && wristCurrentPos <= wristUpPos)
            {
                wristCurrentPos += 0.001;
                wrist.setPosition(wristCurrentPos);
             }
            else if (gamepad2.a && wristCurrentPos >= wristDownPos)
            {
                wristCurrentPos -= 0.001;
                wrist.setPosition(wristCurrentPos);
            }

         // GRIPPER
            if (gamepad2.x)
            {
                gripper.setPosition(0.0);
            }
            else gripper.setPosition(1.0);

            // plane
            if (gamepad2.b)
            {
                plane.setPosition(1.0);
            }
            else plane.setPosition(0.0);
        }
    }
}
