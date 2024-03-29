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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dex.Code;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_leftDrive = null;
    private DcMotor front_rightDrive = null;
    private DcMotor back_leftDrive = null;
    private DcMotor back_rightDrive = null;


     // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_leftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        front_rightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        back_rightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        back_leftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_leftDrive.setDirection(DcMotor.Direction.FORWARD);
        front_rightDrive.setDirection(DcMotor.Direction.REVERSE);
        back_leftDrive.setDirection(DcMotor.Direction.FORWARD);
        back_rightDrive.setDirection(DcMotor.Direction.REVERSE);
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
        double front_leftPower;
        double front_rightPower;
        double back_leftPower;
        double back_rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        front_leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        front_rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        back_leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        back_rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        if (a);
        front_leftDrive.setPower(1);
        if (b);
        front_rightDrive.setPower(1);
        if (x);
        back_leftDrive.setPower(1);
        if (y);
        back_rightDrive.setPower(1);

        // Send calculated power to wheels
        front_leftDrive.setPower(front_leftPower);
        front_rightDrive.setPower(front_rightPower);
        back_leftDrive.setPower(back_leftPower);
        back_rightDrive.setPower(back_rightPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "fleft (%.2f), fright (%.2f), bleft (%.2f), bright (%.2f)", front_leftPower, front_rightPower, back_leftPower, back_rightPower);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
