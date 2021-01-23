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
    double shooterPower = 0;    //power for shooter
    double pulleyPower = 0;     //power for pulley

    public DcMotor shooter;     //wheels to shoot rings
    public DcMotor shooter2;
    public Servo intake1;       //right intake??
    public Servo intake2;       //left intake??

    public DcMotor pulley;      //pulley to lift for shooter

    public DcMotor leftFront;   //drivetrainwheels
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;


    @Override
    public void init() {

        shooter = hardwareMap.dcMotor.get("shooter");
        shooter2 = hardwareMap.dcMotor.get("shooter2");

        intake2 = hardwareMap.servo.get("intake1");
        intake1 = hardwareMap.servo.get("intake2");

        pulley = hardwareMap.dcMotor.get("pulley");

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    @Override
    public void loop() {
        float x;
        float y;
        float z;

        //GAMEPAD 2 ------------------------------------------
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

        if (gamepad1.x) {
            shooterPower = 0.5;

        } else if (gamepad1.y){
            shooterPower = 0;

        }


        //GAMEPAD 1 ------------------------------------------

        //intake
        if((gamepad1.right_trigger) >0.1) {
            intake1.setPosition(intake1.getPosition() + 0.1);

        } else if ((gamepad1.left_trigger) >0.1) {
            intake1.setPosition(intake1.getPosition() - 0.1);

        } else {
            intake1.setPosition(0.5);

        }

        /* pulley lift
        left trigger = goes up
        right trigger = goes down */
        if (Math.abs(gamepad1.left_trigger) > .1) {
            pulleyPower = -1;

        } else if (Math.abs(gamepad1.right_trigger) > .1) {
            pulleyPower = 1;

        } else {
            pulleyPower = 0;

        }


        /*if((gamepad1.right_trigger) >0.1) {
            intake2.setPosition(intake2.getPosition() + 0.1);

        } else if ((gamepad1.left_trigger) >0.1) {
            intake2.setPosition(intake2.getPosition() - 0.1);

        } else {
            intake2.setPosition(0.5);

        }*/

        shooter.setPower(shooterPower);
        shooter2.setPower(shooterPower);

        pulley.setPower(pulleyPower*0.5);

        leftBack.setPower((x - y - z)*.75);
        leftFront.setPower((-x - y -z)*.75);
        rightBack.setPower((-x - y + z)*.75);
        rightFront.setPower((x - y + z)*.75);


    }

}
