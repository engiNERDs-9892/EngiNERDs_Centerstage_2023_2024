package org.firstinspires.ftc.teamcode.Reference_Code.Notes;                                                  /////////
                                                  //Notes//
                                                  /////////



////////////////////////////////////////////////////////////////////////////////////////////////////
// How to call a motor (Init)
//
//                 // This is how we can call the servo throughout the code
//                 private DcMotor ______  = null;
//
//
// Hardware Map (Init)
//
                   // This is how the Driver Hub and the Control Hub talk to each other
//                 _______  = hardwareMap.get(DcMotor.class, "________");
//
//
// Motor Directions (Init / Functions)
//
//          O Forwards
//
//               {
//                   motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
//                   motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
//                   motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
//                   motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
//               }
//
//
//          O Backwards
//
//               {
//                  motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
//                  motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
//                  motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
//                  motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
//               }
//
//
//          O Strafe Left
//
//               {
//                  motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
//                  motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
//                  motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
//                  motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
//               }
//
//
//          O Strafe Right
//
//               {
//                  motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
//                  motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
//                  motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
//                  motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
//               }
////////////////////////////////////////////////////////////////////////////////////////////////////
// Encoders (Opmode / Functions)
//
//
//
//     Step 1. STOP and RESET Encoders to start the Loop
//
//        motor_________.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//     Step 2. Set the motor direction to either Forwards or REVERSE
//
//        motor_________.setDirection(DcMotorSimple.Direction. REVERSE / FORWARDS);
//
//
//
//     Step 3. Set target position for the motor or servo to go to
//
//        motor_________.setTargetPosition(x);
//
//
//
//     Step 4. Have the motors Run to Positon
//
//        motor_________.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//     Step 5. Set the motor power to a value between 0 & 1
//
//        motor_________.setPower(x (0-1) );
//
//
//     Step 6. Create a while loop for the motor so it doesn't stop running while going to position
//        while (opModeIsActive() && motor_________.isBusy()) {}
//
//
//     Step 7. STOP and RESET Encoders to stop the Loop
//
//        motor_________.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    }
////////////////////////////////////////////////////////////////////////////////////////////////////
// Gyro (Init / Opmode)
//
//
//      O Initialize phase
//
//
//          private BNO055IMU       imu         = null;      // Gyro Code
//
//          private double          robotHeading  = 0;
//          private double          headingOffset = 0;
//          private double          headingError  = 0;
//
//
//          //These variable are declared here (as class members) so they can be updated in various methods,
//          // but still be displayed by sendTelemetry()
//
//          private double  targetHeading  = 0;
//          private double  driveSpeed     = 0;
//          private double  turnSpeed      = 0;
//          private double  motorFL_Speed  = 0;
//          private double  motorBL_Speed  = 0;
//          private double  motorFR_Speed  = 0;
//          private double  motorBR_Speed  = 0;
//          private int     motorFL_Target = 0;
//          private int     motorBL_Target = 0;
//          private int     motorFR_Target = 0;
//          private int     motorBR_Target = 0;
//          private int     leftTarget     = 0;
//          private int     rightTarget    = 0;
//
//
//
//
//          // Calculate the COUNTS_PER_INCH for your specific drive train.
//          // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//          // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//          // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//          // This is gearing DOWN for less speed and more torque.
//          // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//
//          static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: GoBILDA 312 RPM Yellow Jacket
//          static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//          static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//          static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//
//          These constants define the desired driving/control characteristics
//          They can/should be tweaked to suit the specific robot drive train.
//          static final double     DRIVE_SPEED             = 0.6;     // Max driving speed for better distance accuracy.
//          static final double     TURN_SPEED              = 0.4;     // Max Turn speed to limit turn rate
//          static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
//                                                                    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
//
//
//          // Define the Proportional control coefficient (or GAIN) for "heading control".
//          // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
//          // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
//          // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
//          static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
//          static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
//
//
//          // Define initialization values for IMU, and then initialize it.
//          BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//          parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//          imu = hardwareMap.get(BNO055IMU.class, "imu");
//          imu.initialize(parameters);
//
//    O - Op Mode is active
//
//
//        // This is code to get the robot to rotate, USE ENCODERS TO DRIVE FORWARDS/BACKWARDS and STRAFING
//
//        // Use this tell the robot to rotate
//        driveStraight(0,0,X (Degree of Heading));
//
//        // Use this to rotate the robot
//        turnToHeading(turnSpeed,X (Desired Rotation));
//
//        // Use this to keep the robot from over rotating. (Longer time means less likely to over rotate
//        holdHeading(turnSpeed,X (Current Heading), X (Time spent to stop the robot from over rotating));
//
//    O- Functions to use during Op Mode
//
//
//        ///////////////////////////////////////
//        //This Is how you run your autonomous//
//        ///////////////////////////////////////
//
//        Autonomous {
//        WRITE YOUR AUTONOMOUS CODE HERE (Include the Gyro + Encoder code here)
//        }
//
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//        //This is how to read what the robot is actually doing, trying to detect if its a hardware issue or software issue//
//        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//        private void sendTelemetry(boolean straight) {
//
//        if (straight) {
//            telemetry.addData("Motion", "Drive Straight");
//            telemetry.addData("Target Pos (Motors)", "%7d:%7d:%7d:%7d", motorFL_Target, motorFR_Target, motorBL_Target, motorBR_Target);
//            telemetry.addData("Actual Pos (Motors)", "%7d:%7d:%7d:%7d",  motorFL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
//                       }
//
//        // Tells us what is happening while the robot is turning using the gyroscope
//        else {
//            telemetry.addData("Motion", "Turning");
//             }
//
//        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
//        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
//        telemetry.addData("Wheel Speeds", "%5.2f : %5.2f", motorFL_Speed, motorFR_Speed, motorBL_Speed, motorBR_Speed);
//        telemetry.update();
//              }
//
//
//        ///////////////////////////////////////////////////////
//        // Use this function to set the current heading to 0//
//        //////////////////////////////////////////////////////
//
//
//        public void resetHeading() {
//
//        //Save a new heading offset equal to the current raw heading.
//        headingOffset = getRawHeading();
//        robotHeading = 0;
//                          }
//
//        ///////////////////////////////////////////////////////
//        // Use this function to calculate the current heading//
//        ///////////////////////////////////////////////////////
//
//
//        Use this function to calculate the current velocity
//        public double getRawHeading() {
//        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle;
//         }
//
//        //////////////////////////////////////////////////////////
//        // Use this function to get the robot to rotate properly//
//        //////////////////////////////////////////////////////////
//
//        public void moveRobot(double drive, double turn) {
//        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
//        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
//
//
//        motorFL_Speed = drive + turn;
//        motorBL_Speed = drive - turn;
//        motorFR_Speed = drive + turn;
//        motorBR_Speed = drive + turn;
//
//        // Scale speeds down if either one exceeds +/- 1.0;
//        double max = Math.max(Math.abs(motorFL_Speed), Math.abs(motorFR_Speed));
//        double max2 = Math.max(Math.abs(motorBL_Speed), Math.abs(motorBR_Speed));
//        if (max > 1.0)
//        {
//            motorFL_Speed /= max;
//            motorFR_Speed /= max;
//        }
//
//        if (max2 > 1.0)
//        {
//            motorBL_Speed /= max;
//            motorBR_Speed /= max;
//          }
//
//        motorFL.setPower(motorFL_Speed);
//        motorFR.setPower(motorFR_Speed);
//        motorBL.setPower(motorBL_Speed);
//        motorBR.setPower(motorBR_Speed);
//          }
//
//
//        ////////////////////////////////////////////////////////////////////
//        // Use this to drive in a straight line - *NOTE WE DON'T USE THIS*//
//        ////////////////////////////////////////////////////////////////////
//
//
//        public void driveStraight(double speed, double distance, double heading) {
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//        // Determine new target position, and pass to motor controller
//        int moveCounts = (int)(distance * COUNTS_PER_INCH);
//        motorFL_Target = motorFL.getCurrentPosition() + moveCounts;
//        motorFR_Target = motorFR.getCurrentPosition() + moveCounts;
//        motorBL_Target = motorBL.getCurrentPosition() + moveCounts;
//        motorBR_Target = motorBR.getCurrentPosition() + moveCounts;
//
//        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        // Set Target FIRST, then turn on RUN_TO_POSITION
//        motorFL.setTargetPosition(motorFL_Target);
//        motorFR.setTargetPosition(motorFR_Target);
//        motorBL.setTargetPosition(motorBL_Target);
//        motorBR.setTargetPosition(motorBR_Target);
//
//        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
//        // Start driving straight, and then enter the control loop
//        speed = Math.abs(speed);
//        moveRobot(speed, 0);
//
//        // keep looping while we are still active, and BOTH motors are running.
//        while (opModeIsActive() && (motorFL.isBusy() && motorBR.isBusy())) {
//
//        // Determine required steering to keep on heading
//        turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//        // if driving in reverse, the motor correction also needs to be reversed
//        if (distance < 0)
//        turnSpeed *= -1.0;
//
//        // Apply the turning correction to the current driving speed.
//        moveRobot(driveSpeed, turnSpeed);
//
//        // Display drive status for the driver.
//        sendTelemetry(true);
//        }
//
//        // Stop all motion & Turn off RUN_TO_POSITION
//        moveRobot(0, 0);
//
//        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        }
//
//
//
//        ////////////////////////////////////////////////////////////////////
//        // Use this to drive in a straight line - *NOTE WE DON'T USE THIS*//
//        ////////////////////////////////////////////////////////////////////
//
//
//        public void turnToHeading(double turnspeed, double heading) {
//
//        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        // Run getSteeringCorrection() once to pre-calculate the current error
//        getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
//
//        // Determine required steering to keep on heading
//        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//        // Clip the speed to the maximum permitted value.
//        turnSpeed = Range.clip(turnSpeed, -turnspeed, turnspeed);
//
//        // Pivot in place by applying the turning correction
//        moveRobot(0, turnSpeed);
//
//        // Display drive status for the driver.
//        sendTelemetry(false);
//        }
//
//        // Stop all motion;
//        moveRobot(0, 0);
//        }
//
//
//
//        //////////////////////////////////////////////////////////////////////////////////
//        // This is how the robot doesn't over rotate while the speed of rotation is high//
///       //////////////////////////////////////////////////////////////////////////////////
//
//
//        public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//
//        // keep looping while we have time remaining.
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//
//        // Determine required steering to keep on heading
//        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//        // Clip the speed to the maximum permitted value.
//        turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//
//        // Pivot in place by applying the turning correction
//        moveRobot(0, turnSpeed);
//
//        // Display drive status for the driver.
//        sendTelemetry(false);
//        }
//
//        // Stop all motion;
//        moveRobot(0, 0);
//        }
//
//
//
//        ///////////////////////////////////////////////////////////////////////////////////////////
//        // Corrects the heading of the robot if it ever gets off track - *NOTE WE DO NOT USE THIS//
//        ///////////////////////////////////////////////////////////////////////////////////////////
//
//        public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
//        targetHeading = desiredHeading;  // Save for telemetry
//
//        // Get the robot heading by applying an offset to the IMU heading
//        robotHeading = getRawHeading() - headingOffset;
//
//        // Determine the heading current error
//        headingError = targetHeading - robotHeading;
//
//        // Normalize the error to be within +/- 180 degrees
//        while (headingError > 180)  headingError -= 360;
//        while (headingError <= -180) headingError += 360;
//
//        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
//        return Range.clip(headingError * proportionalGain, -1, 1);
//    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////