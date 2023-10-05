/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Current_Code.Auto;


import static org.firstinspires.ftc.teamcode.Variables.Functions.Move;
import static org.firstinspires.ftc.teamcode.Variables.Functions.getRawHeading;
import static org.firstinspires.ftc.teamcode.Variables.Functions.getSteeringCorrection;
import static org.firstinspires.ftc.teamcode.Variables.Functions.moveRobot;
import static org.firstinspires.ftc.teamcode.Variables.Functions.resetHeading;
import static org.firstinspires.ftc.teamcode.Variables.Functions.tagToTelemetry;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.Left;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.Middle;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.P_DRIVE_GAIN;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.P_TURN_GAIN;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.Right;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.camera;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.cx;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.cy;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.directions;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.driveSpeed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.fx;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.fy;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.headingError;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.imu;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBL_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBL_Target;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBR_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorBR_Target;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFL_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFL_Target;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFR;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFR_Speed;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.motorFR_Target;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.robotHeading;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.tagOfInterest;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.tagsize;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.targetHeading;
import static org.firstinspires.ftc.teamcode.Variables.Robot_Constants.turnSpeed;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Auto Blue", group="Robot")
//@Disabled
public class Auto_Blue extends LinearOpMode {

    org.firstinspires.ftc.teamcode.Current_Code.Auto.AprilTagDetectionPipeline aprilTagDetectionPipeline;
///////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorFL  = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorBR  = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorBL  = hardwareMap.get(DcMotorEx.class, "motorBL");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);



        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.Current_Code.Auto.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // Set the encoders for closed loop speed control, and reset the heading.
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (!isStarted() && !isStopRequested()) {

            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


            resetHeading();
            // Calls to the Pipline
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    //         QR 11               QR 12              QR 13
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                // If ANY QR is found Display the Green Text in Telemetry with the QR that is found
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                }

                // If NO QR is found Display one of the following Messages in Green Text
                else
                {
                    // If No Tag is seen then display the text below in the telemetry
                    telemetry.addLine("Don't see tag of interest :(");
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());

                    if(tagOfInterest == null)
                    {
                        // If No Tag is seen then display the text below in the telemetry
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        // Code to display in the telemetry of the last seen QR Code
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());


                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                }

            }

            telemetry.update();
            sleep(20);
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        // If a QR is detected, Then display the one it found in the telemetry
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();

        }

        // If no QR Code is detected
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        // All the Code above is how the camera detects the april tage (^-- That code)
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //////////////////////
        // Autonomous  Code //
        //////////////////////

        if (tagOfInterest == null) {
            //put default code here
            Autonomous();
        }

        // The Left is Sleeve 10 (QR Code 10)
        else if (tagOfInterest.id == Left) {
            Autonomous();
        }

        // The Middle is Sleeve 20 (QR Code 20)
        else if (tagOfInterest.id == Middle) {
            Autonomous();

        }


        // The third else or in this case Right is Sleeve 30 (QR Code 30)
        else {
            Autonomous();

        }
    }

    private void Autonomous(){
        resetHeading();

        ///////////////////////////////
        // Cone #1 or Pre-Loaded Cone//
        ///////////////////////////////


        // Moves FORWARDS to Place the cone on the Small Junction
        Move(directions.FORWARDS,5,.8);

        driveStraight(0,0,0);
        turnToHeading(TURN_SPEED, 90);
        holdHeading(TURN_SPEED,90,.5);

        Move(directions.BACKWARDS,5,.8);
    }




    public void driveStraight(double speed, double distance, double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            motorFL_Target = motorFL.getCurrentPosition() + moveCounts;
            motorFR_Target = motorFR.getCurrentPosition() + moveCounts;
            motorBL_Target = motorBL.getCurrentPosition() + moveCounts;
            motorBR_Target = motorBR.getCurrentPosition() + moveCounts;

            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


            // Set Target FIRST, then turn on RUN_TO_POSITION
            motorFL.setTargetPosition(motorFL_Target);
            motorFR.setTargetPosition(motorFR_Target);
            motorBL.setTargetPosition(motorBL_Target);
            motorBR.setTargetPosition(motorBR_Target);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            moveRobot(speed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (motorFL.isBusy() && motorBR.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos (Motors)",  "%7d:%7d:%7d:%7d",      motorFL_Target,  motorFR_Target, motorBL_Target, motorBR_Target);
            telemetry.addData("Actual Pos (Motors)",  "%7d:%7d:%7d:%7d",      motorFL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
        }

        else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds", "%5.2f : %5.2f", motorFL_Speed, motorFR_Speed, motorBL_Speed, motorBR_Speed);
        telemetry.update();
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading

            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    public void turnToHeading(double turnspeed, double heading) {


        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {


            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -turnspeed, turnspeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
}
