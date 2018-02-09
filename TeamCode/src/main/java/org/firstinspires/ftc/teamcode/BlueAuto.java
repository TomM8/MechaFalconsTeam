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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueAuto", group="Autonomous")
//@Disabled
public class BlueAuto extends LinearOpMode {

    /* Declare OpMode members. */
    MainTeleOp robot  = new MainTeleOp();   // Use a the hardware from the TeleOp
    private ElapsedTime runtime = new ElapsedTime();


    static final double  FORWARD_SPEED = 0.6;
    static final double  TURN_SPEED    = 0.5;
    static final double DRIVE_POWER = 1.0;

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        robot.driveWheel1 = hardwareMap.get(DcMotor.class, "driveWheel1");
        robot.driveWheel2 = hardwareMap.get(DcMotor.class, "driveWheel2");
        robot.blockGrabber1 = hardwareMap.get(Servo.class, "blockGrabber1");
        robot.blockGrabber2 = hardwareMap.get(Servo.class, "blockGrabber2");
        robot.ballSensorServo = hardwareMap.get(Servo.class, "ballSensorServo");
        robot.driveWheelSide = hardwareMap.get(DcMotor.class, "driveWheelSide");
        robot.driveWheel3 = hardwareMap.get(DcMotor.class, "driveWheel3");
        robot.driveWheel4 = hardwareMap.get(DcMotor.class, "driveWheel4");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        // You have to reverse one motor, otherwise a power value of 1.0 would make the motors run
        // in different directions. This just makes it more convenient, so you don't have to use 1.0
        // for one motor and -1.0 for the other motor.
//        liftMotor.setDirection(DcMotor.Direction.REVERSE);
//        motorThree.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.driveWheel1.setDirection(DcMotor.Direction.REVERSE);
        robot.driveWheel3.setDirection(DcMotor.Direction.REVERSE);

        robot.ballSensorServo.setPosition(0.75);

        robot.blockGrabber1.setPosition(0.72);
        robot.blockGrabber2.setPosition(0.29);

        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //wait(2000);

        colorSensor.enableLed(true); // Turn the light on for objects and turn it off if sensing color of lit up objects

        //TODO: This is the theoretical autonomous

        double position = robot.ballSensorServo.getPosition();
        robot.ballSensorServo.setPosition(position);

        //driveRight(0.3, 400);

        //robot.ballSensorServo.setPosition(0.10); //TODO: This has to be the position where the sensor is at the balls

        if(colorSensor.blue() > 0){
            turnLeft(0.4, 300);
            //turnRight(0.4, 250);
        }
        else if(colorSensor.red() > 0){
            turnRight(0.4, 300);
            //turnLeft(0.4, 250);
        }
        else if(colorSensor.blue() == 0 && colorSensor.red() == 0){
            stopDriving();
            robot.ballSensorServo.setPosition(0);
        }

        //driveF(0.4, 700);

        //wait(2000);

        colorSensor.red();
        colorSensor.blue();

        telemetry.addData("SensedRedNumber: ", colorSensor.red());
        telemetry.addData("SensedBlueNumber: ", colorSensor.blue());

        robot.ballSensorServo.setPosition(0); //TODO: This will have to be changes to the home position

    }

    public void driveF(double power, int time) throws InterruptedException {
        robot.driveWheel1.setPower(power);
        robot.driveWheel2.setPower(power);
        robot.driveWheel3.setPower(power);
        robot.driveWheel4.setPower(power);
        Thread.sleep(time);
    }
    public void driveR(double power, int time) throws InterruptedException {
        robot.driveWheel1.setPower(-power);
        robot.driveWheel2.setPower(-power);
        robot.driveWheel3.setPower(-power);
        robot.driveWheel4.setPower(-power);
        Thread.sleep(time);
    }

    public void turnRight(double power, int time) throws InterruptedException {
        robot.driveWheel1.setPower(power);
        robot.driveWheel2.setPower(-power);
        robot.driveWheel3.setPower(power);
        robot.driveWheel4.setPower(-power);
        Thread.sleep(time);

    }

    public void turnLeft(double power, int time) throws InterruptedException {
        robot.driveWheel1.setPower(-power);
        robot.driveWheel2.setPower(power);
        robot.driveWheel3.setPower(-power);
        robot.driveWheel4.setPower(power);
        Thread.sleep(time);

    }

    public void driveLeft(double power, int time) throws InterruptedException {
        robot.driveWheelSide.setPower(power);
        Thread.sleep(time);
    }

    public void driveRight(double power, int time) throws InterruptedException {
        robot.driveWheelSide.setPower(-power);
        Thread.sleep(time);
    }

    public void stopDriving() {
        robot.driveWheel1.setPower(0.0);
        robot.driveWheel2.setPower(0.0);
        robot.driveWheel1.setPower(0.0);
        robot.driveWheel2.setPower(0.0);
    }

}
