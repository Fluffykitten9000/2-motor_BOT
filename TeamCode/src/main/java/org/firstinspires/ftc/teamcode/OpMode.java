
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="OP OF EPIC", group="EPIC STUFF")
public class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor ld = null;
    private DcMotor rd = null;

    private double STATE = 0;

    private BNO055IMU imu;

    private double LAST_IMU_ANGLE = 0;

    private double IMUp = 0;

    private double[] ANGLE_WANTED_FOR_MOVEMENT = {0,0};
    private double div = 10;
    private double[] ANGLE_OFFSET = {0,0};

    private ElapsedTime POWER_LEANING_TIME = new ElapsedTime();
    private boolean LEANING_WAY = true;
    private double LEANING_AUTHORITY = 0.05;


    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rd = hardwareMap.get(DcMotor.class, "rd");
        ld = hardwareMap.get(DcMotor.class, "ld");

        ld.setDirection(DcMotor.Direction.FORWARD);
        rd.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        Orientation angles;

        if (gamepad1.dpad_left) STATE-=0.2;
        if (gamepad1.dpad_right) STATE+=0.2;

        if (gamepad1.dpad_up&&Math.round(STATE)==0) div+=0.1;
        if (gamepad1.dpad_down&&Math.round(STATE)==0) div-=0.1;
        if (gamepad1.dpad_up&&Math.round(STATE)==1) ANGLE_OFFSET[0]+=0.05;
        if (gamepad1.dpad_down&&Math.round(STATE)==1) ANGLE_OFFSET[0]-=0.05;
        if (gamepad1.dpad_up&&Math.round(STATE)==2) LEANING_AUTHORITY+=0.002;
        if (gamepad1.dpad_down&&Math.round(STATE)==2) LEANING_AUTHORITY-=0.002;

        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double off = LAST_IMU_ANGLE - angles.secondAngle;

        LAST_IMU_ANGLE = angles.secondAngle;

        double leftDP;
        double rightDP;

        ANGLE_WANTED_FOR_MOVEMENT[0] = gamepad1.left_stick_y/6;
        ANGLE_WANTED_FOR_MOVEMENT[0]-=ANGLE_WANTED_FOR_MOVEMENT[1];

        IMUp+=(off+ANGLE_OFFSET[0]+ANGLE_WANTED_FOR_MOVEMENT[0])/div;

        leftDP = Range.clip((IMUp+ld.getPower())/2, -1.0, 1.0);
        rightDP = Range.clip((IMUp+rd.getPower())/2, -1.0, 1.0);

        ld.setPower(leftDP);
        rd.setPower(rightDP);

        UPDATE_POWER_LEANING_TIME();

        telemetry.addData("leftDP", leftDP);
        telemetry.addData("rightDP", rightDP);
        telemetry.addData("AWFM", ANGLE_WANTED_FOR_MOVEMENT[1]);
        if (Math.round(STATE)==0) {
            telemetry.addData("Dpad STATE", "change div =" + div);
        }
        if (Math.round(STATE)==1) {
            telemetry.addData("Dpad STATE", "change ANGLE_OFFSET =" + ANGLE_OFFSET[1]);
        }
        if (Math.round(STATE)==2) {
            telemetry.addData("Dpad STATE", "change LEANING_AUTHORITY" + LEANING_AUTHORITY);
        }
        telemetry.update();
        ANGLE_OFFSET[1]+=ANGLE_OFFSET[0];
        ANGLE_WANTED_FOR_MOVEMENT[1]+=ANGLE_WANTED_FOR_MOVEMENT[0];
        ANGLE_OFFSET[0]=0;
        ANGLE_WANTED_FOR_MOVEMENT[0]=0;
    }

    @Override
    public void stop() {
    }
    private void UPDATE_POWER_LEANING_TIME() {
        double POWER_IN_ALL = (ld.getPower()+rd.getPower())/2;
        if (POWER_IN_ALL>=0==LEANING_WAY) {
            LEANING_WAY=!LEANING_WAY;
            POWER_LEANING_TIME.reset();
        }
        if (POWER_LEANING_TIME.milliseconds()>=300&&LEANING_WAY) {
            ld.setPower(ld.getPower()-LEANING_AUTHORITY);
            rd.setPower(rd.getPower()-LEANING_AUTHORITY);
        }
        if (POWER_LEANING_TIME.milliseconds()>=300&&!LEANING_WAY) {
            ld.setPower(ld.getPower()+LEANING_AUTHORITY);
            rd.setPower(rd.getPower()+LEANING_AUTHORITY);
        }
    }
}
