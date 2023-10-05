package org.firstinspires.ftc.teamcode.Old_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous(name="Nates_Code_Basic", group="Iterative Opmode")
public class Nates_Code_Basic extends LinearOpMode
{
    private DcMotor motorfl;
    private DcMotor motorfr;
    private DcMotor motorbl;
    private DcMotor motorbr;

    private int in = 89;

    public void runOpMode() {

        motorfl = hardwareMap.dcMotor.get("motorfl");
        motorfr = hardwareMap.dcMotor.get("motorfr");
        motorbl = hardwareMap.dcMotor.get("motorbl");
        motorbr = hardwareMap.dcMotor.get("motorbr");

        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfl.setDirection(DcMotorSimple.Direction.FORWARD);
        motorbr.setDirection(DcMotorSimple.Direction.REVERSE);

        motorfl.setTargetPosition(5*in);
        motorbr.setTargetPosition(5*in);

        motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorfl.setPower(.5);
        motorbr.setPower(.5);

        while (motorfl.isBusy());{

        }

        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
