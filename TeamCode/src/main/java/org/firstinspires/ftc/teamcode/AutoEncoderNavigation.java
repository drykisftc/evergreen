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
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoEncoder", group = "Teaching")
public class AutoEncoderNavigation extends AutoRelic {

    PIDControl encoderDisPID = new PIDControl();

    int movingForwardDistance = 3000;
    int wheelLandMark = 0;
    int turnDegree = 90;

    int convergeCount =0;
    public AutoEncoderNavigation() {

    }

    @Override
    public void init() {
        robot = new HardwareTeaching();
        robot.init(hardwareMap);
        robot.start();

        leftMotors = new DcMotor[1];
        leftMotors[0] = robot.motorLeftWheel;
        rightMotors = new DcMotor[1];
        rightMotors[0] = robot.motorRightWheel;

        navigation = new Navigation(telemetry);
        navigation.pidControlHeading.setKp(0.004);
        navigation.pidControlHeading.setKi(0.002);
        navigation.pidControlHeading.setKd(0.0000001);
        navigation.maxTurnDeltaPower = 0.4;
        navigation.convergeCountThreshold = 6;
        navigation.angleErrorTolerance = 2.1;

        encoderDisPID.setKp(0.0005);
        encoderDisPID.setKi(0.0001);
        encoderDisPID.setKd(0.00001);
        convergeCount =0;

        robot.gyro.calibrate();
    }

    @Override
    public void init_loop () {
        if (robot.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
            telemetry.addData(">", "Wait! Wait! Wait! ");
        }
        else {
            telemetry.addData(">", "Press Start.");
        }
    }

    @Override
    public void start() {
        wheelLandMark = (robot.motorLeftWheel.getCurrentPosition() +
                robot.motorRightWheel.getCurrentPosition()) /2;
    }

    @Override
    public void loop() {
        telemetry.addData("State:" , state);
        telemetry.addData("leftMotor:", leftMotors[0]);
        telemetry.addData("rightMotor:", rightMotors[0]);
        switch (state) {
            case 0:
                // move forward
                int errorDis = (wheelLandMark + movingForwardDistance)
                -(robot.motorLeftWheel.getCurrentPosition() + robot.motorRightWheel.getCurrentPosition())/2;

                if (Math.abs(errorDis) < 50) {
                    convergeCount ++;
                } else
                {
                    convergeCount = 0;
                }

                if (convergeCount > 5) {
                    state = 1;
                    convergeCount = 0;
                }

                double power = navigation.pidControlDistance.update(errorDis, System.currentTimeMillis());
                robot.motorLeftWheel.setPower(power);
                robot.motorRightWheel.setPower(power);

                break;
            default:
                // stop
                robot.motorLeftWheel.setPower(0);
                robot.motorRightWheel.setPower(0);
                robot.stop();
        }

    }

}
