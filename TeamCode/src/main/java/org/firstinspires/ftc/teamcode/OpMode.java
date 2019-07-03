
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="OP OF EPIC", group="EPIC STUFF")
public class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor ld = null;
    private DcMotor rd = null;

    private double STATE = 0;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

    private double LAST_IMU_ANGLE = 0;

    private double IMUp = 0;

    private double powerMULT = 1;
    private double[] ANGLE_WANTED_FOR_MOVEMENT = {0,0};
    private double div = 10;
    private double[] ANGLE_OFFSET = {0,0};

    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rd = hardwareMap.get(DcMotor.class, "rd");
        ld = hardwareMap.get(DcMotor.class, "ld");

        ld.setDirection(DcMotor.Direction.FORWARD);
        rd.setDirection(DcMotor.Direction.REVERSE);

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

        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
        gravity = imu.getGravity();

        if (gamepad1.dpad_left) STATE-=0.1;
        if (gamepad1.dpad_right) STATE+=0.1;

        if (gamepad1.dpad_up&&Math.round(STATE)==0) div+=.1;
        if (gamepad1.dpad_down&&Math.round(STATE)==0) div-=.1;
        if (gamepad1.dpad_up&&Math.round(STATE)==1) ANGLE_OFFSET[0]+=.01;
        if (gamepad1.dpad_down&&Math.round(STATE)==1) ANGLE_OFFSET[0]-=.01;
        if (gamepad1.dpad_up&&Math.round(STATE)==2) powerMULT+=.001;
        if (gamepad1.dpad_down&&Math.round(STATE)==2) powerMULT-=.001;

        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double off = LAST_IMU_ANGLE - angles.secondAngle;

        LAST_IMU_ANGLE = angles.secondAngle;

        double leftDP;
        double rightDP;

        ANGLE_WANTED_FOR_MOVEMENT[0] = gamepad1.left_stick_y/15;
        ANGLE_WANTED_FOR_MOVEMENT[0]-=ANGLE_WANTED_FOR_MOVEMENT[1] ;

        IMUp+=(off/div)+ANGLE_OFFSET[0]+ANGLE_WANTED_FOR_MOVEMENT[0];

        leftDP = Range.clip(IMUp*powerMULT, -1.0, 1.0);
        rightDP = Range.clip(-IMUp*powerMULT, -1.0, 1.0);

        ld.setPower(leftDP);
        rd.setPower(rightDP);

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
            telemetry.addData("Dpad STATE", "change powerMULT =" + powerMULT);
        }
        telemetry.update();
        ANGLE_OFFSET[1]+=ANGLE_OFFSET[0];
        ANGLE_WANTED_FOR_MOVEMENT[1]+=ANGLE_WANTED_FOR_MOVEMENT[0];
        ANGLE_OFFSET[0]=0;
        ANGLE_WANTED_FOR_MOVEMENT[0]=0;

        imu.stopAccelerationIntegration();
    }

    @Override
    public void stop() {
    }
}
