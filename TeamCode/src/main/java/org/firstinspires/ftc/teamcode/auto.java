package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="auto test", group="auto without IMU")
public class auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fld = null;
    private DcMotor frd = null;
    private DcMotor bld = null;
    private DcMotor brd = null;

    @Override
    public void runOpMode() {
        frd = hardwareMap.get(DcMotor.class, "frd");
        fld = hardwareMap.get(DcMotor.class, "fld");
        brd = hardwareMap.get(DcMotor.class, "brd");
        bld = hardwareMap.get(DcMotor.class, "bld");
        fld.setDirection(DcMotor.Direction.FORWARD);
        fld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frd.setDirection(DcMotor.Direction.REVERSE);
        frd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bld.setDirection(DcMotor.Direction.FORWARD);
        bld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brd.setDirection(DcMotor.Direction.REVERSE);
        brd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Starting");
        telemetry.update();
        doYourSTUFF();
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    // do the crap
    public void doYourSTUFF() {
        if (opModeIsActive()) {
            runtime.reset();
            turn(1000,1000,2000,2000);
        }
    }

    // the driving stuff
    public void drive(int fldTIC,int bldTIC,int brdTIC,int frdTIC) {
        while (fld.getCurrentPosition() - fldTIC != 0 && frd.getCurrentPosition() - frdTIC != 0 && bld.getCurrentPosition() - bldTIC != 0 && brd.getCurrentPosition() - brdTIC != 0) {
            double a = Range.clip(fldTIC - fld.getCurrentPosition(), -1.0, 1.0);
            fld.setPower(a);
            double b = Range.clip(frdTIC - frd.getCurrentPosition(), -1.0, 1.0);
            frd.setPower(b);
            double c = Range.clip(bldTIC - bld.getCurrentPosition(), -1.0, 1.0);
            bld.setPower(c);
            double d = Range.clip(brdTIC - brd.getCurrentPosition(), -1.0, 1.0);
            brd.setPower(d);
        }
    }
    public void turn(int fldTIC,int bldTIC,int brdTIC,int frdTIC) {
        while (fld.getCurrentPosition() - fldTIC != 0 && frd.getCurrentPosition() - frdTIC != 0 && bld.getCurrentPosition() - bldTIC != 0 && brd.getCurrentPosition() - brdTIC != 0) {
            int all = fldTIC + bldTIC + brdTIC + frdTIC;
            double fldP = fldTIC/all;
            double frdP = frdTIC/all;
            double bldP = bldTIC/all;
            double brdP = brdTIC/all;
            telemetry.addData("num", fldP);
            telemetry.update();
            fld.setPower(fldTIC/all);
            frd.setPower(frdP);
            bld.setPower(bldP);
            brd.setPower(brdP);
        }
    }
}