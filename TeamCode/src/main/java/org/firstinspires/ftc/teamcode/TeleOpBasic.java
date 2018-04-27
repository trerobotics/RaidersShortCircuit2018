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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="RaiderBot: Worlds", group="RaiderBot")
//@Disabled
public class TeleOpBasic extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RaiderBot robot = new RaiderBot(telemetry);

    private float conveyorSpeed = .7f;
    private float maxSpeed = .6f;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap);

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
        float gamePad1LeftYJoystick = -gamepad1.left_stick_y;
        float gamePad1LeftXJoystick = gamepad1.left_stick_x;
        float gamePad1RightXJoystick = gamepad1.right_stick_x;
        float gamePad1RightTrigger = gamepad1.right_trigger;
        float gamepad1Lefttrigger = gamepad1.left_trigger;
        float gamePad2RightTrigger = gamepad2.right_trigger;
        float gamePad2LeftTrigger = gamepad2.left_trigger;
        boolean gamePad2AButton = gamepad2.a;
        boolean gamePad2DPadUp = gamepad2.dpad_up;
        boolean gamePad2DPadDown = gamepad2.dpad_down;
        boolean gamePad2X = gamepad2.x;

        drive(gamePad1LeftXJoystick, gamePad1LeftYJoystick, gamePad1RightXJoystick);

        robot.runConveyor(gamePad2LeftTrigger + -gamePad2RightTrigger);

        robot.moveRelic(gamepad1Lefttrigger - gamePad1RightTrigger);

        if(gamePad2DPadUp)
        {
            robot.RelicUp();
        }

        if(gamePad2DPadDown)
        {
            robot.RelicDown();
        }

        if(gamePad2AButton){
            robot.toggleRelicFinger();
        }

        if(gamePad2X)
        {
            robot.flipblock();
        }
    }

    public void drive(float xInput, float yInput, float zInput)
    {
        robot.FrDrive.setPower(Range.clip((yInput - xInput - zInput), -maxSpeed, maxSpeed));
        robot.FlDrive.setPower(Range.clip((yInput + xInput + zInput), -maxSpeed, maxSpeed));
        robot.BrDrive.setPower(Range.clip((yInput + xInput - zInput), -maxSpeed, maxSpeed));
        robot.BlDrive.setPower(Range.clip((yInput - xInput + zInput), -maxSpeed, maxSpeed));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
