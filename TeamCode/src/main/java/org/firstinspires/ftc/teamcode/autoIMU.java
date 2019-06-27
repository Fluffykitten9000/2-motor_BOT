//Hi! Java rocks!
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.sql.Time;
import java.text.DecimalFormat;
import java.text.Format;
import java.text.NumberFormat;
import java.util.Timer;

@Autonomous(name="IMU test", group="auto with IMU")
public class autoIMU extends LinearOpMode {

    //the references to the motors
    private DcMotor fld = null;
    private DcMotor frd = null;
    private DcMotor bld = null;
    private DcMotor brd = null;
    //the accelerometer calibrate data
    private double acc[] = new double[3];
    private double accOff[] = new double[3];

    //timer
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {
        //set motor hardwareMap
        frd = hardwareMap.get(DcMotor.class, "frd");
        fld = hardwareMap.get(DcMotor.class, "fld");
        brd = hardwareMap.get(DcMotor.class, "brd");
        bld = hardwareMap.get(DcMotor.class, "bld");

        //set imu map and settings
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //set motor direction
        fld.setDirection(DcMotor.Direction.FORWARD);
        frd.setDirection(DcMotor.Direction.REVERSE);
        bld.setDirection(DcMotor.Direction.FORWARD);
        brd.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "calibrating");
        telemetry.update();

        //stop and get ready for epicness (reset encoders)
        fld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //run using encoders
        fld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intACC();

        //done calibrating message
        telemetry.addData("Status", "done calibrating");
        telemetry.update();

        //wait for start
        waitForStart();
        runtime.reset();
        //message for start
        telemetry.addData("Status", "Starting");
        telemetry.update();
        doYourSTUFF();
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    //this holds all of what it will do in auto
    public void doYourSTUFF() {
        if (opModeIsActive()) {
            //gyroDrive(0.2, 10000,1000);
            gyroTurn(0.5, 90, 20000);
        }
    }

    //drive using imu gyro
    public void gyroDrive(double speed, double time, double distance) {
        //number that tells the max rot in a frame
        double MAX_SCALE_ANGLE = 90;
        //number that tells the scale factor so we don't get in a feed back loop of doom
        double SCALED_NUM = 5;
        //distance from start forward
        double MY_DISTANCE = 0;
        intACC();
        //loop that makes shore that its on track
        while (opModeIsActive()&&runtime.milliseconds()<time&&MY_DISTANCE<=distance) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            //power the motors----------------------\/----i don't know why it needs this to keep the front motors the same pace as the back
            fld.setPower(Range.clip(speed + 0.8 + Range.scale(angles.thirdAngle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -1, 1));
            frd.setPower(Range.clip(speed + 0.8 + -Range.scale(angles.thirdAngle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -1, 1));
            bld.setPower(Range.clip(speed + 0 + Range.scale(angles.thirdAngle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -1, 1));
            brd.setPower(Range.clip(speed + 0 + -Range.scale(angles.thirdAngle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -1, 1));
            MY_DISTANCE+=ACC()[1];
            // tell driver whats going on
            telemetry.addLine()
                    .addData("distance", MY_DISTANCE);
            telemetry.addLine()
                    .addData("acc", ACC()[1]);
            telemetry.update();
        }
        //set motor power back to 0
        fld.setPower(0);
        frd.setPower(0);
        bld.setPower(0);
        brd.setPower(0);
    }
    
    public void gyroTurn(double speed,double angle,double time) {
        angle *= -1;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        //number that tells the max rot in a frame
        double MAX_SCALE_ANGLE = 180;
        //number that tells the scale factor so we don't get in a feed back loop of doom
        double SCALED_NUM = 10;
        intACC();
        double smallAngle = Math.round(angles.thirdAngle);//Double.parseDouble(String.format("%.1f", angles.thirdAngle));
        //loop that makes shore that its on track
        while (opModeIsActive()&&runtime.milliseconds()<time&&smallAngle-angle>1) {
            smallAngle = Math.round(angles.thirdAngle);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            //power the motors
            fld.setPower(Range.clip(Range.scale(angles.thirdAngle + angle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -speed, speed));
            frd.setPower(Range.clip(-Range.scale(angles.thirdAngle + angle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -speed, speed));
            bld.setPower(Range.clip(Range.scale(angles.thirdAngle + angle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -speed, speed));
            brd.setPower(Range.clip(-Range.scale(angles.thirdAngle + angle, -MAX_SCALE_ANGLE, MAX_SCALE_ANGLE, -SCALED_NUM, SCALED_NUM), -speed, speed));
            telemetry.addData(">",smallAngle);
            telemetry.update();
        }
        //set motor power back to 0
        fld.setPower(0);
        frd.setPower(0);
        bld.setPower(0);
        brd.setPower(0);
    }

    // both of these are for help with the accelerometer
    public void intACC() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
        gravity = imu.getGravity();
        acc[0] = gravity.xAccel;
        acc[1] = gravity.yAccel;
        acc[2] = gravity.zAccel;
        accOff[0] = gravity.xAccel;
        accOff[1] = gravity.yAccel;
        accOff[2] = gravity.zAccel;
        imu.stopAccelerationIntegration();
    }
    public double[] ACC() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
        gravity = imu.getGravity();
        acc[0] = gravity.xAccel;
        acc[1] = gravity.yAccel;
        acc[2] = gravity.zAccel;
        imu.stopAccelerationIntegration();
        return new double[]{acc[0]-accOff[0], acc[1]-accOff[1], acc[2]-accOff[2]};
    }
}
