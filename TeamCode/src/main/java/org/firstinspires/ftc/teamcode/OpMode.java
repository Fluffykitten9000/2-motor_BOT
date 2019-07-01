
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

    private  BNO055IMU imu;

    private Orientation angles;

    private double LAST_IMU_ANGLE = 0;

    double SCALE = 4;

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

        if (gamepad1.dpad_up) SCALE+=.01;
        if (gamepad1.dpad_down) SCALE-=.01;

        double SCALED = 1;

        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double IMUp = LAST_IMU_ANGLE - angles.secondAngle;

        LAST_IMU_ANGLE = angles.secondAngle;

        double leftDP;
        double rightDP;

        double leftUP = gamepad1.left_stick_y;
        double rightUP = gamepad1.right_stick_y;

        if (IMUp>-1&&IMUp<0.1) IMUp=0;

        leftDP = Range.clip(leftUP + Range.scale(IMUp, -SCALE, SCALE, -SCALED, SCALED), -1.0, 1.0);
        rightDP = Range.clip(rightUP - Range.scale(IMUp, -SCALE, SCALE, -SCALED, SCALED), -1.0, 1.0);

        ld.setPower(leftDP);
        rd.setPower(rightDP);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("leftDP", leftDP);
        telemetry.addData("rightDP", rightDP);
        telemetry.addData("SCALE", SCALE);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
