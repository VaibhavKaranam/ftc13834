/* Copyright (c) 4017 FIRST. All rights reserved.
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


@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class TeleOpMech extends OpMode
{
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor mechArm = null;
    private DcMotor clamp = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.dcMotor.get("leftMotor");
        rightDrive = hardwareMap.dcMotor.get("rightMotor");
        mechArm = hardwareMap.dcMotor.get("armMotor");
        clamp = hardwareMap.dcMotor.get("clamp");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    //Code runs repeatedly after driver presses INIT
    @Override
    public void init_loop() {
    }

    //Code runs once Driver presses PLAY
    @Override
    public void start() {

        runtime.reset();
    }

    //Code repeatedly runs when pressed PLAY and stops when pressed STOP
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad2.right_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double leftY2 = -gamepad2.left_stick_y;
        double rightY2 = -gamepad2.left_stick_y;

        // Tank Mode uses one stick to control each wheel
        //send calculated power to the motors
        leftDrive.setPower(leftY + rightX);
        rightDrive.setPower(leftY - rightX);
        mechArm.setPower(leftY2);
        clamp.setPower(rightY2 * 0.25);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Left Power", leftDrive.getPower());
        telemetry.addData("Right Power", rightDrive.getPower());
        telemetry.addData("Arm Power", mechArm.getPower());
        telemetry.addData("Clamp Power", clamp.getPower());
    }

    //code stops once driver presses STOP
    @Override
    public void stop() {
    }

}