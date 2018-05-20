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

@Autonomous(name = "AutoDistance", group = "Teaching")
public class AutoDistanceMecanum extends AutoRelic {

    protected HardwareMecanum robot= null;

    PIDControl encoderDisPID = new PIDControl();

    int movingForwardDistance = 3000;
    int wheelLandMark = 0;
    int turnDegree = 90;

    int convergeCount =0;
    public AutoDistanceMecanum() {

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


        encoderDisPID.setKp(0.0005);
        encoderDisPID.setKi(0.000001);
        encoderDisPID.setKd(0.000);
        encoderDisPID.setMaxIntegralError(0.002f/ encoderDisPID.fKi);
        convergeCount =0;
    }

    @Override
    public void init_loop () {
    }

    @Override
    public void start() {
        wheelLandMark = getMovingDistance();
        convergeCount =0;
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
                // move forward
                int distance = getMovingDistance();
                telemetry.addData("Distance:" , distance);
                int errorDis = (wheelLandMark + movingForwardDistance) - distance;
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

                double power = encoderDisPID.update(errorDis, System.currentTimeMillis());
                setMovingPower(Range.clip(power,-1,1));

                break;
            default:
                // stop
                setMovingPower(0);
                wheelLandMark = getMovingDistance();
                convergeCount =0;
                return 0;
        }
        return 1;
    }

    protected int getMovingDistance () {
        int distance = 0;

        for (int i = 0; i < leftMotors.length; i++) {
            distance += leftMotors[i].getCurrentPosition();
        }

        for (int i = 0; i < rightMotors.length; i++) {
            distance += rightMotors[i].getCurrentPosition();
        }
        return distance/(leftMotors.length+rightMotors.length);
    }

    protected void setMovingPower (double p) {

        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i].setPower(p);
        }

        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i].setPower(p);
        }
    }


}
