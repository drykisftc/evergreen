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

@Autonomous(name = "AutoDistancePIDSearch", group = "Teaching")
public class AutoDistancePIDSearch extends AutoDistanceMecanum {

    int aiState =0;

    // TODO: change searching range
    double [] Ps = { 0.0005, 0.005, 0.01 };
    double [] Is = { 0.0001, 0.001, 0.01 };
    double [] Ds = { 0.00000, 0.00001, 0.0001 } ;

    int pIndex =0;
    int iIndex =0;
    int dIndex = 0;

    long pidStartTS = 0;
    long pidEndTS = 0;
    long shortestT = 0;

    double bestP, bestI, bestD;

    @Override
    public void start() {
        super.start();
        aiState =0;
        pIndex =0;
        iIndex =0;
        dIndex = 0;
        shortestT = Long.MAX_VALUE;
        bestP = 0;
        bestI = 0;
        bestD = 0;
    }

    @Override
    public void init_loop () {
        // TODO: use game pad to adjust PID searching ranges
    }

    @Override
    public void loop() {
        if (pIndex < Ps.length) {
            telemetry.addData("best Kp=", Ps[pIndex]);
        }
        if (iIndex < Is.length) {
            telemetry.addData("best ki=", Is[iIndex]);
        }
        if (dIndex < Ds.length) {
            telemetry.addData("best kd=", Ds[dIndex]);
        }
        switch (aiState)
        {
            case 0:
                super.start();
                // TODO: loop through PID parameters
                if (Ps.length >= pIndex) {
                    pIndex = 0;
                }
                if (Is.length >= iIndex) {
                    iIndex = 0;
                }
                if (Ds.length >= dIndex) {
                    dIndex = 0;
                }

                // set pid parameters
                encoderDisPID.setKp(Ps[pIndex]);
                encoderDisPID.setKi(Is[iIndex]);
                encoderDisPID.setKd(Ds[dIndex]);
                encoderDisPID.setMaxIntegralError(0.002f/ encoderDisPID.fKi);

                // start timer
                pidStartTS = System.currentTimeMillis();

                aiState = 1;

                break;
            case 1:
                if ( 0== execute() ){ // embedded state machine
                    aiState = 2;
                    pidEndTS = System.currentTimeMillis();
                }
                break;
            case 2:
                // compute performance
                if (shortestT > pidEndTS - pidStartTS ) {
                    bestP = Ps[pIndex];
                    bestI = Is[iIndex];
                    bestD = Ds[dIndex];
                    shortestT = pidEndTS - pidStartTS;
                }

                aiState = 3;
                // flip distance
                movingForwardDistance *= -1;

                break;
            case 3:
                // find out the best PID parameter
                telemetry.addData("best Kp=", bestP);
                telemetry.addData("best ki=", bestI);
                telemetry.addData("best kd=", bestD);

                // TODO: index handling
                dIndex ++;

                if (dIndex >= Ds.length) {
                    iIndex++;
                }
                if (iIndex >= Is.length) {
                    pIndex ++;
                }

                if ( dIndex >= Ds.length && iIndex >= Is.length && pIndex >= Ps.length) {
                    aiState = 4;
                } else {
                    aiState = 0;
                }

                break;

            default:
                telemetry.addData("best Kp=", bestP);
                telemetry.addData("best ki=", bestI);
                telemetry.addData("best kd=", bestD);
                telemetry.addData("Done", "!");
        }
    }


}
