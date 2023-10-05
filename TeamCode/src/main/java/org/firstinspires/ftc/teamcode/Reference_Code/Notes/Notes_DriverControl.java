package org.firstinspires.ftc.teamcode.Reference_Code.Notes;                                                  /////////
                                                  //Notes//
                                                  /////////



////////////////////////////////////////////////////////////////////////////////////////////////////
// Calling Servos and Motors
//
//      // This is how we can call the servo throughout the code
//      Servo _________;
//
//      // This is how we can call the servo throughout the code
//      private DcMotor ______  = null;
//
//
//
// Hardware Map (Init)
//
//                 // This is how the Driver Hub and the Control Hub talk to each other
//                 _______  = hardwareMap.get(DcMotor.class, "________");
//                 _______ = hardwareMap.servo.get("_______");
//
//
// Servo Directions (Opmode / Function)
//
//          O Forwards
//
//               {
//                 _______.setDirection(Servo.Direction.FORWARDS);
//               }
//
//
//          O Reverse
//
//               {
//                _______.setDirection(Servo.Direction.REVERSE);
//               }
//
//
//
// Continuous Servos V.S. Regular servos (NOTES)
//
//          // Continuous Servos - Do not stop rotating once a direction is picked
//          // Regular Servos - Only rotate to a specific position
//
//
//
//  How to set a position for the servo to rotate to (Opmode/ Function)
//
//          // This is how we rotate the Regular Servos
//          ________.setPosition(X);
//
//
//
//  Motor Directions (Init / Functions)
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
//
//
//   How to assign buttons to actions (* Gamepad - TeleOp)
//
//
//      O Boolean - (Single button thats True / False)
//
//
//           if (gamepad2.a) {
//           // Action goes Here
//            }
//
//
//      O Triggers / No boolean actions
//
//            if (gamepad1.left_trigger != 0) {
//
//            // Action goes here
//
//            }
//
//      O Joystick values and how to use them
//
//        -  Basic Joysick use
//
//               // You have to have 4 if else statements (1st is if it has a positive value
//               // Then the 2nd one is if its positive and the trigger is not pressed, then its if it is positive and pressed
//               // Then the last is if its less than the positive value or negative value
//
//
//            // This is the action and that if the value on joystick is negative and the sensor is not tripped (Typically going up)
//            if (RaiseorLower < -0.05){
//
//            // Action goes here
//
//            }
//
//
//            // This is the action and that if the value on joystick is positive and the sensor is not tripped (Typically going down)
//            if (gamepad__.left_stick_y  > 0.05 && !touchB.isPressed()){
//
//            // Action goes here
//
//            // This is the action and that if the value on joystick is positive and the sensor is not tripped
//            if (RaiseorLower > 0.05 && touchB.isPressed() ){
//
//              // Action goes here
//
//            }
//        }
//
//
//            if((RaiseorLower >= -0.05) && (RaiseorLower <= 0.05)){
//
//                  // Action goes here
//            }
//        }
//
//
//
//
//         - Math joystick
//          (Variables)
//
//         double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//
//         // The code below talks about the X-axis (Left and Right)
//
//         double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed
//
//         // The code below talks about Z-Axis (Spinning around)
//
//         double yaw = -gamepad1.right_stick_x;
//
//         // Variable used in the math section
//         double max;
//
//
//           (Math)
//
//           double leftFrontPower = .6 * (axial + lateral + yaw);
//           double rightFrontPower = .6 * (axial - lateral - yaw);
//           double leftBackPower = .6 * (axial - lateral + yaw);
//           double rightBackPower = .6 * (axial + lateral - yaw);
//
//
//            // This calculates the direction & power for Regular Speed
//            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));
//
//
//            // sets the wheels to do whatever the calculation above tells it to do for Regular Speed
//            if (max > 1.0) {
//                leftFrontPower /= max;
//                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
//              }
//
//              // Sets motor power based on the Controller input
//               else {
//                motorFL.setPower(leftFrontPower);
//                motorBL.setPower(leftBackPower);
//                motorFR.setPower(rightFrontPower);
//                motorBR.setPower(rightBackPower);
//               }
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////