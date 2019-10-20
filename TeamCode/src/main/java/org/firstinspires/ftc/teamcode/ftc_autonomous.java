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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="ftc_autonomous", group="Linear Opmode")
//Disabled
public class ftc_autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    private DcMotor rightback = null;

    private double drivepower = 0.0;
    private double right = 0.0;
    private double left = 0.0;

    //This function is used to make robot go forward or backward based on the drive power (-0.5: Backward), (0.5: Forward), (0:Stop)
    private void forwardBackward() {
        leftfront.setPower(drivepower);
        rightfront.setPower(drivepower);
        rightback.setPower(drivepower);
        leftback.setPower(drivepower);
    }

    //Before using this function set the variable(left) to 1
    // This function would be used when you want to move your robot towards the left.
    // Example: left = 1
    private void lateralLeft() {
        leftfront.setPower(-left);
        rightfront.setPower(left);
        rightback.setPower(-left);
        leftback.setPower(left);
    }

    //Before using this function set the variable(right) to 1
    // This function would be used when you want to move your robot towards the right.
    // Example: right = 1
    private void lateralRight() {
        leftfront.setPower(-right);
        rightfront.setPower(right);
        rightback.setPower(-right);
        leftback.setPower(right);
    }

    //This function is used for setting the direction of the motors(for rightward and leftward motion)
    private void setMovementLateral() {
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.FORWARD);
    }

    // This function is used for setting the direction of the motors(for forward and backward motion)
    private void setMovement_For_Back() {
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);
    }

    //Define class members
    Servo   servo;
    double  servoPosition = 0.4; // Start at halfway position
    boolean rampUp = true;



        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            servo = hardwareMap.servo.get("servo");
            servo.setPosition(servoPosition);

            // Initialize the hardware variables. Note that the strings used here as parameters
            leftfront = hardwareMap.get(DcMotor.class, "leftfront");
            rightfront = hardwareMap.get(DcMotor.class, "rightfront");
            rightback = hardwareMap.get(DcMotor.class, "rightback");
            leftback = hardwareMap.get(DcMotor.class, "leftback");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            setMovement_For_Back();

            waitForStart();
            runtime.reset();


            //drive backward for 2 seconds
            drivepower = -0.5;
            forwardBackward();
            telemetry.addData("Status", "Moving Backward");
            telemetry.update();
            sleep(1770);

            //stop motors
            drivepower = 0.0;
            forwardBackward();
            telemetry.addData("Status", "Stopping");
            telemetry.update();

            // servomotors
            servoPosition = 0.95;
            servo.setPosition(servoPosition);
            sleep(900);

            //drive forwards for 2 seconds
            drivepower = 0.2;
            leftfront.setPower(0.8);
            rightfront.setPower(0.3);
            leftback.setPower(0.8);
            rightback.setPower(0.3);
            telemetry.addData("Status", "Moving Forward");
            telemetry.update();
            sleep(2900);

            drivepower = 0.0;
            forwardBackward();
            telemetry.addData("Status", "Stop Program");
            telemetry.update();

            servoPosition = 0.4;
            servo.setPosition(servoPosition);
            sleep(500);
            drivepower = 0.0;


            // Reverse the motor that runs backwards when connected directly to the battery //Lateral Movement
            setMovementLateral();

            waitForStart();
            runtime.reset();

            left = 1;
            lateralLeft();
            telemetry.addData("Status", "Stop Program");
            telemetry.update();
            sleep(2200);

            telemetry.addData("Status", "Stop Program");
            telemetry.update();
        }
    }



