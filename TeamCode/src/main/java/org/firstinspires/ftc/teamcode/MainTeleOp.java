

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * CONTROL SCHEME
 *
 * Gamepad 1:
 * Motor one and motor two - left joystick
 * Motor three and motor four - right joystick
 * Color sensor enable - X
 *
 *
 * TeleOp for the 2017/18 FTC games
 * --> Motor/s for the lifting of the arms to extend to the height for placing the blocks
 * --> Servos to extend outwards and inwards to grab the blocks
 * --> Driving motors (Done)
 */

// It's a teleop. It ops the tele. Simple.
// This is the teleOp for the new and improved robot!!!
@TeleOp(name = "TheTeleOp", group = "Iterative Opmode")
public class MainTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // These are constants, once you set them you cannot change them
    // Nice for settings things like motor power
    // This way you can use names instead of "magic numbers"
    //static final double MOTOR_FULL_POWER = 1.0;
    static final double MOTOR_LESS_POWER = 0.7;
    static final double MOTOR_FULL_POWER = 1.0;
    static final double MOTOR_HALF_POWER = 0.5;
    static final double MOTOR_LEAST_POWER = 0.3;
    static final double MOTOR_POWER_OFF = 0.0;
    static final double SERVO_MAX = 1.0;
    static final double SERVO_MID = 0.5;
    static final double INCREMENT_SPEED = 0.02;
    double blockGrabberPosition = 0.42;
    double blockGrabberPosition2 = 0.59;
    double ballServoPosition;

//    HardwareMap hwMap = null;

    // Defining your motors - DcMotor is a class provided by the FTC SDK (software dev kit)
    DcMotor liftMotor;
    DcMotor driveWheel1;
    DcMotor driveWheel2;
    DcMotor driveWheelSide;
    DcMotor driveWheel3;
    DcMotor driveWheel4;
    Servo blockGrabber1;
    Servo blockGrabber2;
    Servo ballSensorServo;
    ColorSensor colorSensor;


    // Similarly, if you wanted to define a servo, you would put:
    // Servo servoName;
    // The servoName can be anything you want, it's a object.


    /*
     * This runs when the driver station "init" button is pressed
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");



        /*
         * Use hardwareMap.dcMotor.get() to map the variable initialized above to the motor.
         * The argument in quotes is the name of the motor. You set this in the robot profile
         * on the robot controller phone.
         */
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        driveWheel1 = hardwareMap.get(DcMotor.class, "driveWheel1");
        driveWheel2 = hardwareMap.get(DcMotor.class, "driveWheel2");
        blockGrabber1 = hardwareMap.get(Servo.class, "blockGrabber1");
        blockGrabber2 = hardwareMap.get(Servo.class, "blockGrabber2");
        ballSensorServo = hardwareMap.get(Servo.class, "ballSensorServo");
        driveWheelSide = hardwareMap.get(DcMotor.class, "driveWheelSide");
        driveWheel3 = hardwareMap.get(DcMotor.class, "driveWheel3");
        driveWheel4 = hardwareMap.get(DcMotor.class, "driveWheel4");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        // You have to reverse one motor, otherwise a power value of 1.0 would make the motors run
        // in different directions. This just makes it more convenient, so you don't have to use 1.0
        // for one motor and -1.0 for the other motor.
//        liftMotor.setDirection(DcMotor.Direction.REVERSE);
//        motorThree.setDirection(DcMotorSimple.Direction.REVERSE);
        driveWheel1.setDirection(DcMotor.Direction.REVERSE);
        driveWheel3.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ballSensorServo.setPosition(0.1);
        blockGrabber1.setPosition(0.72);
        blockGrabber2.setPosition(0.29);

    }


    /*
     * This runs after the "init" button is pressed but before the "play" button is pressed
     */
    @Override
    public void init_loop() {
    }

    //Hello

    /*
     * This runs when the "play" button is pressed on the driver station
     */
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    // This runs after the "play" button is pressed and before the "stop" button is pressed
    public void loop() {

        //colorSensor.enableLed(true);

        //colorSensor.red();
        //colorSensor.blue();

        //telemetry.addData("sensing red value: ", colorSensor.red());
        //telemetry.addData("sensing blue value: ", colorSensor.blue());

        // If you put the cursor on a comment that says "region" and press command-minus, you can collapse the code

        //region Telemetry

        // Telemetry == stuff that shows up on the driver station phone screen
        telemetry.addData("Status", "Running: " + runtime.toString());
       // telemetry.addData("Wheels", "1: " + motorOne.getPower() + " 2: " + motorTwo.getPower() + " 3: " + motorThree.getPower() + "4: " + motorFour.getPower());

        //endregion

        //region Wheels


        //This is the code for the driving wheels
        //The turning responds to the two gamepad triggers
        if(gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0){
            driveWheel1.setPower(MOTOR_HALF_POWER);
            driveWheel2.setPower(-MOTOR_HALF_POWER);
            driveWheel3.setPower(MOTOR_HALF_POWER);
            driveWheel4.setPower(-MOTOR_HALF_POWER);
        }
        else if(gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
            driveWheel1.setPower(-MOTOR_HALF_POWER);
            driveWheel2.setPower(MOTOR_HALF_POWER);
            driveWheel3.setPower(-MOTOR_HALF_POWER);
            driveWheel4.setPower(MOTOR_HALF_POWER);
        }
        else {
            //Joystick values trigger forward and backward motion
            driveWheel1.setPower(joystickToMotorValue(-gamepad1.left_stick_y)/0.5);
            driveWheel2.setPower(joystickToMotorValue(-gamepad1.left_stick_y)/0.5);
            driveWheel3.setPower(joystickToMotorValue(-gamepad1.left_stick_y)/0.5);
            driveWheel4.setPower(joystickToMotorValue(-gamepad1.left_stick_y)/0.5);
            driveWheelSide.setPower(joystickToMotorValue(-gamepad1.left_stick_x));
        }


        //endregion

        //region Lifting motor

        //Lift motor runs with an encoder

        //&& Math.abs(liftMotor.getCurrentPosition()

        int minHeight = liftMotor.getCurrentPosition();
        int maxHeight = minHeight+8961;

        if (!gamepad1.a) {
            if (gamepad1.y && liftMotor.getCurrentPosition() < maxHeight) {
                liftMotor.setPower(MOTOR_LESS_POWER);
            } else if (gamepad1.b && liftMotor.getCurrentPosition() > minHeight) {
                liftMotor.setPower(-MOTOR_HALF_POWER);
            } else {
                liftMotor.setPower(MOTOR_POWER_OFF);
            }
        } else {
            liftMotor.setPower(-MOTOR_LESS_POWER);
        }


        telemetry.addData("Lift Motor", + liftMotor.getPower());
        telemetry.addData("Encoder position: ", minHeight);

        //TODO: lowest encoder value = -2718, highest encoder value = 6243


        //endregion

        //region Block grabbing servos

        if(gamepad1.right_bumper){
            blockGrabber1.setPosition(0.40);
            blockGrabber2.setPosition(0.70);
        }
        else {
            blockGrabber1.setPosition(0.72);
            blockGrabber2.setPosition(0.29);
        }

        //Initial values for servo:
            // BlockGrabber = 0.72;
            // BlockGrabber2 = 0.29
        //Final values for servo:
            // BlockGrabber = 0.32;
            // BlockGrabber2 = 0.69
        //TODO: Find the final values for the servo



        //endregion

        //region ball Sensing Mechanism
        //TODO: The reason this doesn't work is because it wants to set to the zero position which it can't access
        //TODO: To fix it, you have to see where it is at zero without it being attached to the robot and the roughly input a value which it can access as home
        /*if(gamepad1.dpad_down){
            ballServoPosition -= INCREMENT_SPEED;
        }
        else if(gamepad1.dpad_up) {
            ballServoPosition += INCREMENT_SPEED;
        }

        ballSensorServo.setPosition(ballServoPosition); //Here I give the servos the blockGrabberPosition values

        telemetry.addData("ballSensorServo", "%.2f", ballServoPosition); //Important --> shows the values of the servo on the phone
        //telemetry.addData("blockGrabber2", "%.2f", blockGrabberPosition2);*/

        if(gamepad1.x){
            ballSensorServo.setPosition(0.8);
        }
        else {
            ballSensorServo.setPosition(0.1);
        }

        //TODO: This might have been done, but we have to change the servo to be places horizontally
        double ServoPosition = ballSensorServo.getPosition();

        telemetry.addData("Servo Position: ", ServoPosition);



        //endregion

    }

    /*
     * This runs once after the stop button is pressed on the driver station
     */
    @Override
    public void stop() {
    }

    // This scales the input so that we maintain precision control
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    // This clips the values from the joystick so that they can be used for the motor.
    double joystickToMotorValue(double joystickValue) {
        joystickValue = Range.clip(joystickValue, (float) -1.0, (float) 1.0);
        double scaled = scaleInput(joystickValue); // This scales input for some reason
        return scaled;
    }

}
