package org.firstinspires.ftc.teamcode.Variables;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.driveSpeed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.headingError;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.headingOffset;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.imu;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.in;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBL_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBR_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFL_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFR;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFR_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.robotHeading;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.targetHeading;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.turnSpeed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;


public class Functions {

    public static void Move(Robot_Constants.directions direction, int target, double speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // This sets the direction for the motor for the wheels to drive forward
        if (direction == Robot_Constants.directions.FORWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move Backwards
        else if (direction == Robot_Constants.directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == Robot_Constants.directions.LEFT) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == Robot_Constants.directions.RIGHT) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Gives it a position to run to
        motorFL.setTargetPosition(target * in);
        motorFR.setTargetPosition(target * in);
        motorBL.setTargetPosition(target * in);
        motorBR.setTargetPosition(target * in);

        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // the motor speed for Wheels
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);


        // While loop keeps the code running until motors reach the desired position
        while (((motorFL.isBusy() || motorFR.isBusy()))) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public static double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public static void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    public static double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }




    public static void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    public static void Park_R(){
    }

    public static void Park_L(){
    }

    public static void Park_C(){
    }
    public static void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        motorFL_Speed = drive - turn;
        motorBL_Speed = drive - turn;
        motorFR_Speed = drive + turn;
        motorBR_Speed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(motorFL_Speed), Math.abs(motorFR_Speed));
        double max2 = Math.max(Math.abs(motorBL_Speed), Math.abs(motorBR_Speed));
        if (max > 1.0)
        {
            motorFL_Speed /= max;
            motorFR_Speed /= max;
        }

        if (max2 > 1.0)
        {
            motorBL_Speed /= max;
            motorBR_Speed /= max;
        }

        motorFL.setPower(motorFL_Speed);
        motorFR.setPower(motorFR_Speed);
        motorBL.setPower(motorBL_Speed);
        motorBR.setPower(motorBR_Speed);
    }
}