package org.firstinspires.ftc.teamcode.Variables;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;


public class Robot_Constants {


    // Distances for driving with encoders\

    // Travels 2 feet
    public static double Feet6 = 72;
    public static double Feet5 = 60;
    public static double Feet4 = 48;
    public static double Feet3 = 38;
    public static double Feet2 = 25;
    public static double Feet1 = 14;

    public static double Inches6 = 8;

    //////////////////////////////////////
    // CAMERA VARIABLES | DO NOT TOUCH  //
    //////////////////////////////////////

    public static org.firstinspires.ftc.teamcode.Variables.Robot_Constants AprilTagDetectionPipeline;
    public static AprilTagDetection tagOfInterest = null;

    public static OpenCvCamera camera;

    // UNITS ARE METERS
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
    public static double tagsize = 0.166;

    // Parking Positions and what APRIL TAG they are
    public static int Left = 11; //Detects april tag id#11 - Attached to sleeve template position one
    public static int Middle = 12; //Detects april tag id#12 - Attached to sleeve template position two
    public static int Right = 13; //Detects april tag id#13 - Attached to sleeve template position three

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////
    // MOTOR NAMES   | FUNCTIONS              //
    ////////////////////////////////////////////

    // Motor Names
    public static DcMotorEx motorFR, motorFL, motorBL, motorBR = null;

    // Encoder distance for robot
    public static int in = 45;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////
    // GYROSCOPE / IMU                            //
    ////////////////////////////////////////////////

    // This is the gyroscope or a internal IMU
    public static BNO055IMU imu = null;

    // Variable for Gyroscope

    public static double robotHeading  = 0;
    public static double headingOffset = 0;
    public static double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    public static double  targetHeading  = 0;
    public static double  driveSpeed     = 0;
    public static double  turnSpeed      = 0;
    public static double  motorFL_Speed  = 0;
    public static double  motorBL_Speed  = 0;
    public static double  motorFR_Speed  = 0;
    public static double  motorBR_Speed  = 0;
    public static int     motorFL_Target = 0;
    public static int     motorBL_Target = 0;
    public static int     motorFR_Target = 0;
    public static int     motorBR_Target = 0;
    public static int     leftTarget     = 0;
    public static int     rightTarget    = 0;



    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.6;     // Max driving speed for better distance accuracy.
    public static final double     TURN_SPEED              = 0.4;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.


    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Linear slide Variable (Might not use this)
    public static int up = 145;

    public static enum directions {
        FORWARDS,
        BACKWARDS,
        LEFT,
        RIGHT,
    }
}