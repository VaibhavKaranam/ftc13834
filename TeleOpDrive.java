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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tank Drive TeleOp", group="Linear Opmode")
public class TeleOpDrive extends LinearOpMode {

    // Declare OpMode variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private double y1 = 0.;
    private double y2 = 0.;
    private double speed = 0.5;

@Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initializing hardware variables left motor and right motor
        leftMotor  = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        // Setting direction of motors.
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for driver to press START
        waitForStart();
        runtime.reset();

        // Run until pressed STOP end of the game essentially
        while (opModeIsActive()) {
            y1 = gamepad1.left_stick_y;
            y2 = gamepad1.right_stick_y;
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("rightMotor: " + y2, "leftMotor: " + y1);
            telemetry.addData("Speed: ", speed);
            telemetry.update();

            //Tank Mode; joysticks go negative when pushed forward so we need to add minus
            //forward movement
            leftMotor.setPower(-y1 * speed);
            rightMotor.setPower(-y2 * speed);

            if(gamepad1.b){
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            //If left bumper is pressed then speed decreases, if right bumper is pressed then speed increases
            //set speed of motor using left and right bumpers
            if(gamepad1.left_bumper){
                speed -= 0.1;
                wait(0.5);
                if(speed < 0.1){
                    speed = 0.1;
                }
            }else if(gamepad1.right_bumper) {
                speed += 0.1;
                wait(0.5);
                if (speed > 1) {
                    speed = 1;
                }
            }
        }
    }
    public void wait(double seconds){
        double time = this.time;
        while(this.time - time < seconds){
            //to wait for a certain amount of seconds
        }
    }
}