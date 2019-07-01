/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="OP OF EPICNESSSSSS", group="EPIC STUFF")
public class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fld = null;
    private DcMotor frd = null;
    private DcMotor bld = null;
    private DcMotor brd = null;

    BNO055IMU imu;

    Orientation angles;

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

        frd = hardwareMap.get(DcMotor.class, "frd");
        fld = hardwareMap.get(DcMotor.class, "fld");
        brd = hardwareMap.get(DcMotor.class, "brd");
        bld = hardwareMap.get(DcMotor.class, "bld");

        fld.setDirection(DcMotor.Direction.REVERSE);
        frd.setDirection(DcMotor.Direction.REVERSE);
        bld.setDirection(DcMotor.Direction.FORWARD);
        brd.setDirection(DcMotor.Direction.REVERSE);

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

        angles = imu.getAngularOrientation()

        double leftDP;
        double rightDP;

        double rightIMUp = ;
        double leftIMUp;

        double leftUP = gamepad1.left_stick_y;
        double rightUP = gamepad1.right_stick_y;

        leftDP = Range.clip(leftUP + leftIMUp, -1.0, 1.0);
        rightDP = Range.clip(rightUP + rightIMUp, -1.0, 1.0);

        fld.setPower(leftDP);
        frd.setPower(rightDP);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("leftDP", leftDP);
        telemetry.addData("rightDP", rightDP);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
