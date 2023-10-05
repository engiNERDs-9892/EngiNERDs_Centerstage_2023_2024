package org.firstinspires.ftc.teamcode.Reference_Code.Notes;                                                  /////////
                                                  //Notes//
                                                  /////////



////////////////////////////////////////////////////////////////////////////////////////////////////
// Camera Calling / April Tags (Init)
//
//          // This is how to call the Camera through out the code
//          OpenCvCamera camera;
//
//          // This is how the Camera knows where to pull information from (THIS IS A MUST FOR THE CODE TO WORK)
//          AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//
// Hardware Map (Init)
//
//          // This is how the Driver Hub and the Control Hub talk to each other
//          int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//          camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), cameraMonitorViewId);
//
//
// Setting the Parameters (Init)
//
//          // This sets the paramaters for the Camera and what to look for during autonomous (Typically goes below the Hardware map)
//          aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.Current_Code.Auto.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//          // Tells the camera to use aprilTagDetectionPipline (Goes below the parameters)
//          camera.setPipeline(aprilTagDetectionPipeline);
//
//
// How to change the ID/QR Codes
//
//    int Left = 11; //Detects april tag id#11 - Attached to sleeve template position 1
//    int Middle = 12; //Detects april tag id#12 - Attached to sleeve template position 2
//    int Right = 13; //Detects april tag id#13 - Attached to sleeve template position 3
//
//
//
// Telling the robot what "Tag to look for" + Orientation of the camera + Telemetry (Init)
//
//            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//
//            // Tells the camera the view and also the Orientation
//            camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//            });
//
//            telemetry.setMsTransmissionInterval(50);
//            while (!isStarted() && !isStopRequested()) {
//
//            // Calls to the Pipline
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//
//                    // Tells the machine to detect a specific QR code
//                          QR 11               QR 12              QR 13
//                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//
//
// Telemetry for Camera + Gyro (Init)
//       .
//                // If ANY QR is found Display the Green Text in Telemetry with the QR that is found
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//                }
//
//                // If NO QR is found Display one of the following Messages in Green Text
//                else
//                {
//                    // If No Tag is seen then display the text below in the telemetry
//                    telemetry.addLine("Don't see tag of interest :(");
//                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//
//                    if(tagOfInterest == null)
//                    {
//                        // If No Tag is seen then display the text below in the telemetry
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        // Code to display in the telemetry of the last seen QR Code
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//
//
//                          }
//                      }
//                  }
//               else
//               {
//                telemetry.addLine("Don't see tag of interest :(");
//                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//                          }
//
//                      }
//              telemetry.update();
//              sleep(20);
//              }
//              if(tagOfInterest != null)
//              {
//              telemetry.addLine("Tag snapshot:\n");
//              tagToTelemetry(tagOfInterest);
//              telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//              telemetry.update();
//              Close_Claws();
//
//              }
//
//              // If no QR Code is detected
//              else
//              {
//              telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//              telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//              Close_Claws();
//              telemetry.update();
//               }
//
//
//
//
//  Function to use the camera properly (Functions)
//
//              // This updates the information that the camera is seeing
//              void tagToTelemetry(AprilTagDetection detection) {
//              telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//              }
//
//
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////