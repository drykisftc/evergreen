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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "AutoVuforiaMecanum", group = "Teaching")
public class AutoVuforiaMecanum extends AutoRelic {

    protected HardwareMecanum robot= null;
    protected HardwareVuforia vuforia = null;

    PIDControl xDisPID = new PIDControl();
    PIDControl yDisPID = new PIDControl();

    int movingForwardDistance = 3000;
    int wheelLandMark = 0;
    int turnDegree = 90;
    int convergeCount =0;

    double targetX =2;
    double targetY = 12;
    double targetThreshold = 0.5;

    public AutoVuforiaMecanum() {

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


        xDisPID.setKp(0.0005);
        xDisPID.setKi(0.000001);
        xDisPID.setKd(0.000);
        xDisPID.setMaxIntegralError(0.002f/ xDisPID.fKi);

        yDisPID.setKp(0.0005);
        yDisPID.setKi(0.000001);
        yDisPID.setKd(0.000);
        yDisPID.setMaxIntegralError(0.002f/ yDisPID.fKi);
        convergeCount =0;

        vuforia = new HardwareVuforia(VuforiaLocalizer.CameraDirection.BACK);

    }
    @Override
    public void stop() {
        super.stop();
        vuforia.stop();
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
                // get  x y distance
                OpenGLMatrix pose = vuforia.getGlyphCryptoPosition();
                if (null == pose) {
                    state = 1; // turn
                } else {
                    telemetry.addData("Pose", format(pose));

                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC,
                            AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    //double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    double tD = getVuforiaFrontBackDistance(pose);
                    double tG = getVuforiaLeftRightDistance(pose);
                    telemetry.addData("vuforia distance X=", trans.get(0));
                    telemetry.addData("vuforia distance Y=", tY);
                    telemetry.addData("vuforia distance Z=", tZ);
                    telemetry.addData("vuforia degree 1=", rot.firstAngle);
                    telemetry.addData("vuforia degree 2=", rot.secondAngle);
                    telemetry.addData("vuforia degree 3=", rot.thirdAngle);
                    telemetry.addData("Wall distance   =", tD);
                    telemetry.addData("Image distance   =", tG);

                    double xError = targetX - tG;
                    double yError = targetY - tD;

                    if ( Math.abs(xError) < targetThreshold && Math.abs(yError)< targetThreshold) {
                        state = 2;
                    } else {

                        if (Math.abs(xError) > Math.abs(yError)) {
                            // move x
                            double power = xDisPID.update(xError, System.currentTimeMillis());
                            sideMoveAtPower(power);

                        } else {
                            // move y
                            double power = yDisPID.update(yError, System.currentTimeMillis());
                            moveAtPower(power);
                        }
                    }
                }

                break;
            case 1:
                // turn to find target image
                setMovingPower (-0.2, 02);
                state = 0;
                break;
            default:
                // stop
                setMovingPower(0);

                state = 0;
                return 0;
        }
        return 1;
    }

    public void sideMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(-p);
        robot.motorRightBackWheel.setPower(-p);
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    public void rightDiagonalMoveAtPower(double p) {
        robot.motorRightFrontWheel.setPower(p);
        robot.motorRightBackWheel.setPower(0.0);
        robot.motorLeftBackWheel.setPower(p);
        robot.motorLeftFrontWheel.setPower(0.0);
    }

    public void leftDiagonalMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(0.0);
        robot.motorRightBackWheel.setPower(p);
        robot.motorRightFrontWheel.setPower(0.0);
    }

}
