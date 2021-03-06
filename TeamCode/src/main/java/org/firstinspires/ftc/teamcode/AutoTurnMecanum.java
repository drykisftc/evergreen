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
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "AutoTurnMecanum", group = "Teaching")
public class AutoTurnMecanum extends AutoRelic {

    protected HardwareMecanum robot= null;

    int movingForwardDistance = 3000;
    int wheelLandMark = 0;
    int turnDegree = 90;

    public AutoTurnMecanum() {

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

        navigation = new Navigation(telemetry);
        navigation.pidControlHeading.setKp(0.004);
        navigation.pidControlHeading.setKi(0.004);
        navigation.pidControlHeading.setKd(0.0000001);
        navigation.pidControlHeading.setMaxIntegralError(0.2f/navigation.pidControlHeading.fKi);
        navigation.maxTurnDeltaPower = 1.0;
        navigation.convergeCountThreshold = 6;
        navigation.angleErrorTolerance = 1.1;

    }

    @Override
    public void init_loop () {
    }

    @Override
    public void start() {
        wheelLandMark = getMovingDistance();
        navigation.resetTurn(leftMotors, rightMotors);
        state = 0;
    }

    @Override
    public void loop() {
        execute();
    }

    public int execute() {
        telemetry.addData("State:" , state);

        switch (state) {
            case 0:
                // turn by gyro
                if (0 == navigation.turnByGyroCloseLoop(0, robot.imu.getAngularOrientation().firstAngle, turnDegree,
                        leftMotors, rightMotors)) {
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    wheelLandMark = getMovingDistance();
                    state = 1;
                }
                break;
            default:
                // stop
                setMovingPower(0);
                navigation.resetTurn(leftMotors, rightMotors);
                getWheelLandmarks();
                wheelLandMark = getMovingDistance();
                return 0;
        }
        return 1;
    }

}
