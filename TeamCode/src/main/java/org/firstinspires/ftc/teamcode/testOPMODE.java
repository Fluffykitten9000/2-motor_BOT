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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OP OF EPICNESSSSSS", group="EPIC STUFF")
public class testOPMODE extends com.qualcomm.robotcore.eventloop.opmode.OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fld = null;
    private DcMotor frd = null;
    private DcMotor bld = null;
    private DcMotor brd = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frd = hardwareMap.get(DcMotor.class, "frd");
        fld = hardwareMap.get(DcMotor.class, "fld");
        brd = hardwareMap.get(DcMotor.class, "brd");
        bld = hardwareMap.get(DcMotor.class, "bld");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fld.setDirection(DcMotor.Direction.FORWARD);
        frd.setDirection(DcMotor.Direction.REVERSE);
        bld.setDirection(DcMotor.Direction.FORWARD);
        brd.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double fldP;
        double frdP;
        double bldP;
        double brdP;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        fldP = Range.clip(drive + turn + strafe, -1.0, 1.0);
        frdP = Range.clip(drive - turn - strafe, -1.0, 1.0);
        bldP = Range.clip(drive + turn - strafe, -1.0, 1.0);
        brdP = Range.clip(drive - turn + strafe, -1.0, 1.0);

        fld.setPower(fldP);
        frd.setPower(frdP);
        bld.setPower(bldP);
        brd.setPower(brdP);
    }
        private void setSpeed(DcMotor MOTOR_REFERENCE, double SPEED) {
            long MILLISECOND_SLEEP_FOR_MOTOR_SPEED = 5;
            long THIS_MOTORS_SPEED;
            long PAST_MOTOR_POSITION = MOTOR_REFERENCE.getCurrentPosition();
            long NOW_MOTOR_POSITION = MOTOR_REFERENCE.getCurrentPosition();
            THIS_MOTORS_SPEED=PAST_MOTOR_POSITION-NOW_MOTOR_POSITION;
            telemetry.addData("THIS_MOTORS_SPEED",THIS_MOTORS_SPEED);
            telemetry.update();
        }

    @Override
    public void stop() {
    }
}
