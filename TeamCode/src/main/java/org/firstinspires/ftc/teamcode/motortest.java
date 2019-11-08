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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="motortest", group="Iterative Opmode")
public class motortest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    hardware_v1 robot       = new hardware_v1();
    double          clawOffset  = 0.3 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;
    double          arm_position = 0.7;
    double          claw_rotation = 0.21;
    double          lift_height = 0;
    double          claw_position;


     // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        robot.rotationClaw.setPosition(claw_rotation);
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
        double intake_power = 0;
        double lift_power;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        front_leftPower = Range.clip(drive + turn, -1.0, 1.0);
        front_rightPower = Range.clip(drive - turn, -1.0, 1.0);
        back_leftPower = Range.clip(drive + turn, -1.0, 1.0);
        back_rightPower = Range.clip(drive - turn, -1.0, 1.0);


        if (gamepad1.b) {
            robot.grabClaw.setPosition(.3);
        } else {
            robot.grabClaw.setPosition(.4);
        }


        //intake controls

        if (gamepad1.x) {
            robot.rightIntake.setPower(1);
            robot.leftIntake.setPower(1);
            intake_power = 1;
        } else {
            robot.rightIntake.setPower(0);
            robot.leftIntake.setPower(0);
            intake_power = 0;
        }

     //claw controls
 /*  if (gamepad1.b) {
        if (clawOffset != 1) {
            clawOffset = .7;
        } else if (clawOffset == 1) {
            clawOffset = 0.4;
        }


    }
 */



//arm rotation
    if (gamepad1.a) {
        if (arm_position == .7) {
            arm_position = .9;
        } else if (arm_position == .9)
            arm_position = .7;
        }



//claw rotation
        if (gamepad1.y) {
            if (claw_rotation == 0.21) {
                claw_rotation = .8;
            }
                claw_rotation = 0.21;

        }

//lift control

        if (gamepad1.dpad_up) {
            lift_height += 0.1;
            lift_power = 0.1;
        } else if (gamepad1.dpad_down) {
            lift_height -= 0.1;
            lift_power = -0.1;
        } else {
            lift_power = 0;
        }


    // Send calculated power to wheels

    robot.frontleftDrive.setPower(front_leftPower);
    robot.frontrightDrive.setPower(front_rightPower);
    robot.backleftDrive.setPower(back_leftPower);
    robot.backrightDrive.setPower(back_rightPower);
    robot.rotationClaw.setPosition(arm_position);
    robot.grabClaw.setPosition(arm_position);
    robot.liftControl.setPower(lift_power);

    // Show the elapsed game time and wheel power.
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Motors", "fleft (%.2f), fright (%.2f), bleft (%.2f), bright (%.2f)", front_leftPower, front_rightPower, back_leftPower, back_rightPower);
    telemetry.addData("Claw", "power (%.2f)", clawOffset);
    telemetry.addData("Intake Power", "power (%.2f)", intake_power);
    telemetry.addData("Claw Rotation", "Position (%.2f)  ", claw_rotation);
    telemetry.addData("Arm Position", "Position (%.2f)", arm_position);
    telemetry.addData("Lift Height", "Lift Height (%.2f)", lift_height);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
