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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.*;

@Autonomous(name = "AutoRangeStop", group = "Teaching")
public class AutoRangeStop extends AutoRelic {
    final int measureCounter = 20;
    double avgMeasurement;
    long lasttime;
    double lastdistance;

    HardwareMecanum robot = null;

    PIDControl rangePID = new PIDControl();

    int convergeCount =0;

    ModernRoboticsI2cRangeSensor range = null;

    double targetDistance = 5;

    long lastTime;

    public AutoRangeStop() {

    }

    @Override
    public void init() {
        robot = new HardwareMecanum();
        robot.init(hardwareMap);
        robot.start();

        leftMotors = new DcMotor[2];
        leftMotors[0] = robot.motorLeftFrontWheel;
        leftMotors[1] = robot.motorLeftBackWheel;
        rightMotors = new DcMotor[2];
        rightMotors[0] = robot.motorRightFrontWheel;
        rightMotors[1] = robot.motorRightBackWheel;

        rangePID.setKp(0.06);
        rangePID.setKi(0.0008);
        rangePID.setKd(0.00000);
        rangePID.setMaxIntegralError(0.06f/rangePID.fKi);

        convergeCount =0;

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

    }

    @Override
    public void init_loop () {

        if (gamepad1.x) {
            targetDistance += 0.1;
        }

        if (gamepad1.y) {
            targetDistance -= 0.1;
        }

        telemetry.addData("Target Distance:" , targetDistance);

    }

    @Override
    public void start() {
        rangePID.reset();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        telemetry.addData("State:" , state);
        switch (state) {
            case 1:
                double totalMeasurement = 0;
                for (int i = 0; i < measureCounter; i++) {
                    totalMeasurement += range.getDistance(DistanceUnit.INCH);
                }
                avgMeasurement = totalMeasurement / measureCounter;
                lastdistance = avgMeasurement;
                lasttime = System.currentTimeMillis();
                telemetry.addData("Average distance: ", avgMeasurement);
                state = 0;
                break;
            case 0:
                // move forward
                double d = range.getDistance(DistanceUnit.INCH);
                telemetry.addData("current Distance:" , d);

                if (!Double.isNaN(d)) {
                    long time = System.currentTimeMillis();
                    telemetry.addData(Long.toString(time) + ": ", d);
                    double slope = (d - lastdistance) / (time - lasttime);
                    if (convergeCount > 5) {
                        state = 1;
                        convergeCount = 0;
                    } else {

                        long currentTime = System.currentTimeMillis();
                        double power
                                = com.qualcomm.robotcore.util.Range.clip(-rangePID.update(d, currentTime - lastTime), -1, 1);
                        lastTime = currentTime;

                        telemetry.addData("Power:", power);

                        for (int i = 0; i < leftMotors.length; i++) {
                            leftMotors[i].setPower(power);
                        }

                        for (int i = 0; i < rightMotors.length; i++) {
                            rightMotors[i].setPower(power);
                        }
                    }

                }
                break;
            default:
                // stop
                for (int i =0; i < leftMotors.length; i++) {
                    leftMotors[i].setPower(0);
                }

                for (int i =0; i < rightMotors.length; i++) {
                    rightMotors[i].setPower(0);
                }
                state = 0;
                rangePID.reset();
                //robot.stop();
        }

    }

}
