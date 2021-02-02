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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="teleop", group="Iterative Opmode")
public class teleop extends OpMode

{
    boolean changed = false;
    double shooterPower = 0;    //power for shooter
    double pulleyPower = 0;     //power for pulley
    double intakePower = 0;     //power for intake

    //motors------------------------------------
    public DcMotor leftFront;   //drivetrainwheels
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor shooter;     //wheels to shoot rings
    public DcMotor shooter2;

    public DcMotor pulley;      //pulley to lift for shooter

    public DcMotor intake; //power intake

    //servos----------------------------------
    public Servo pinwheel1;       //right intake to straighten ring
    public Servo pinwheel2;       //left intake to straighten ring
    public Servo pinwheel3;       // bottom pinwheel???
    public Servo pinwheel4;

    public Servo lift;  //wobble goal lifter
    public Servo lift2;
    public Servo lift3;

    public Servo claw; //wobble goal clasp/claw

    public Servo flicker; //pushes ring to shooter

    @Override
    public void init() {

        shooter = hardwareMap.dcMotor.get("shooter");
        shooter2 = hardwareMap.dcMotor.get("shooter2");

        pinwheel1 = hardwareMap.servo.get("pinwheel1");
        pinwheel2 = hardwareMap.servo.get("pinwheel2");
       pinwheel3 = hardwareMap.servo.get("pinwheel3");
        pinwheel4 = hardwareMap.servo.get("pinwheel4");

        pulley = hardwareMap.dcMotor.get("pulley");

       leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.servo.get("lift");
        lift2= hardwareMap.servo.get("lift2");
        lift3= hardwareMap.servo.get("lift3");

        intake = hardwareMap.dcMotor.get("intake");

        claw = hardwareMap.servo.get("claw");

        flicker = hardwareMap.servo.get("flicker");
    }
    @Override
    public void loop() {
        float x;
        float y;
        float z;

        //GAMEPAD 2 ------------------------------------------
        //drivetrain
        if (Math.abs(gamepad2.left_stick_x) > .1) {
            x = gamepad2.left_stick_x;
        } else {
            x = 0;
        }

        if (Math.abs(gamepad2.left_stick_y) > .1) {
            y = -gamepad2.left_stick_y;
        } else {
            y = 0;
        }

        if (Math.abs(gamepad2.right_stick_x) > .1) {
            z = -gamepad2.right_stick_x;
        } else {
            z = 0;
        }

         /*   pulley lift
        left trigger = goes up
        right trigger = goes down */

        if (Math.abs(gamepad2.left_trigger) > .1) {
            pulleyPower = -1;

        } else if (Math.abs(gamepad2.right_trigger) > .1) {
            pulleyPower = 1;

        } else {
            pulleyPower = 0;

        }

        //shooter
        if (gamepad2.x) {
            shooterPower = 1;

        } else if (gamepad2.y){
            shooterPower = 0;

        }

        //GAMEPAD 1 ------------------------------------------

        //INTAKE--------------------------------------------

        //intake pinwheels (moves in opposite directions)
        if((gamepad1.right_trigger) >0.1) {
            pinwheel2.setPosition(pinwheel2.getPosition() + 0.1);
            pinwheel3.setPosition(pinwheel3.getPosition() + 0.1);
            pinwheel1.setPosition(pinwheel1.getPosition() - 0.1);
            pinwheel4.setPosition(pinwheel4.getPosition() - 0.1);

        } else if ((gamepad1.left_trigger) >0.1) {
            pinwheel2.setPosition(pinwheel2.getPosition() - 0.1);
            pinwheel3.setPosition(pinwheel3.getPosition() - 0.1);
            pinwheel4.setPosition(pinwheel4.getPosition() + 0.1);
            pinwheel4.setPosition(pinwheel4.getPosition() + 0.1);

        } else {
            pinwheel2.setPosition(0.5);
            pinwheel3.setPosition(0.5);
            pinwheel1.setPosition(0.5);
            pinwheel4.setPosition(0.5);
        }
        //powers intake
        if (Math.abs(gamepad1.left_trigger) > .1) {
            intakePower = -1;

        } else if (Math.abs(gamepad1.right_trigger) > .1) {
            intakePower = 1;

        } else {
            intakePower = 0;
        }

        //flicker moves back and forth 90 degrees continuous
        if ((gamepad1.right_trigger)>0.1) {
            flicker.setPosition(flicker.getPosition() + 0.1);
        } else if ((gamepad1.left_trigger) >0.1) {
            flicker.setPosition(flicker.getPosition() - 0.1);
        } else {
            flicker.setPosition(0.5);
        }

        //WOBBLE GOAL-------------------------------
        //41 degrees???
        if (gamepad2.a) {
            lift.setPosition(0.22);
            lift2.setPosition(0.22);
            lift3.setPosition(0.22);
        }

        //180 degrees
        if (gamepad2.b) {
            lift.setPosition(1);
            lift2.setPosition(1);
            lift3.setPosition(1);
        }

        //back to set position
        if (gamepad2.x) {
            lift.setPosition(0);
            lift2.setPosition(0);
            lift3.setPosition(0);
        }

        //open close claw
        if(gamepad2.y && !changed) {
            claw.setPosition(1);
            changed = true;
        } else if (!gamepad1.y) {
            changed = false;
        }

        shooter.setPower(shooterPower);
        shooter2.setPower(shooterPower);

       pulley.setPower(pulleyPower*0.5);

        intake.setPower(intakePower);

       leftBack.setPower((x - y - z)*.75);
        leftFront.setPower((-x - y -z)*.75);
        rightBack.setPower((-x - y + z)*.75);
        rightFront.setPower((x - y + z)*.75);


    }
}

