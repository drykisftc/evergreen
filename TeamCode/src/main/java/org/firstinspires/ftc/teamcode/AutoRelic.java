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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Random;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Nathen_PlanA_Red", group = "Hidden")
@Disabled
public class AutoRelic extends OpMode {

    protected String teamColor = "red";


    Random rand = new Random(System.currentTimeMillis());

    protected int state;
    protected long timeStamp;
    protected float wheelDistanceLandMark;
    protected float wheelDistanceAverage;
    protected int columnDistance;
    protected int leftBackStamp;
    protected int rightBackStamp;

    protected float fGlyphTurnAngle = -85; // positive turns left, negative turns right
    protected float fGlyphTurnAngle2 = 0;
    protected double glyTurnPower = -0.8;
    protected double glyTurnPowerLow = -0.5;
    protected float fCenterTurnAngle = 180;


    protected double jewelArmPos = 0;
    protected double jewelHitterPos = 0;
    protected double glyphMovePower = 0.5;
    protected double sideMovePower = 0.4;
    protected double vuforiaDetectingPower = 0.4;
    protected double move2GlyphBoxPower = 0.2;
    protected double glyphBackupPower = 0.2;
    protected double center2GlyphBoxPower = -0.9;
    protected double move2CenterPower = 0.9;
    protected double collectingGlyphPower = 0.5;
    protected double glyphDeliverPower = 0.2;
    protected double backupPower = -0.1;

    protected int cryptoBoxStopDistance = 20;
    protected int offBalanceStoneDistance = 2500;
    protected int rightColumnDistance = 2550;
    protected int centerColumnDistance = 3200;
    protected int leftColumnDistance = 3850;
    protected int cryptoBoxDistance = 500;
    protected int backupDistance = -100;
    protected int pushDistance = 600;
    protected int glyph2CenterDistance = 3200;
    protected int center2GlyphDistance = 3080;
    protected float glyphOffAngle = 0;

    float centerGlyphAngleOffset = 0;

    protected double liftMotorHolderPower = 0.3;
    int glyphLiftPosition = 0;
    protected double liftMotorMovePower = 0.8;
    protected int liftMoveMotorPosition = 400;
    protected int liftMoveMotorPosition2 = 4000;

    protected double lastLeftPower = 0;
    protected double lastRightPower = 0;

    protected JewelKicker jewelKicker = null;

    protected Navigation navigation = null;

    protected HardwareVuforia vuforia = null;

    protected ColorSensor jewelSensor = null;
    protected DistanceSensor jewelSensorDistance = null;

    protected Servo jewelArm = null;
    protected Servo jewelHitter = null;
    protected Servo smolL = null;

    DcMotor[] leftMotors;
    DcMotor[] rightMotors;

    @Override
    public void init() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
    }

    public String computeGlyphColumnDistance() {
        vuforia.identifyGlyphCrypto();
        if (vuforia.vumarkImage == "left") {
            columnDistance = leftColumnDistance;
            glyphOffAngle = 20;
        } else if (vuforia.vumarkImage == "center") {
            columnDistance = centerColumnDistance;
            glyphOffAngle = -10;
        } else if (vuforia.vumarkImage == "right") {
            columnDistance = rightColumnDistance;
            glyphOffAngle = -20;
        } else {
            columnDistance = rightColumnDistance;
            glyphOffAngle = -20;
        }

        OpenGLMatrix pose = vuforia.getGlyphCryptoPosition();
        telemetry.addData("Pose", format(pose));

        return vuforia.vumarkImage;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void moveAtPower(double p) {

        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i].setPower(p);
        }
        lastLeftPower = p;


        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i].setPower(p);
        }
        lastRightPower = p;

    }

    public int moveByDistance(double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        } else if (power > 0) {
            if (getWheelOdometer() - wheelDistanceLandMark < distance) {
                moveAtPower(power);
            } else {
                moveAtPower(0.0);
                return 0;
            }
        } else {
            if (wheelDistanceLandMark - getWheelOdometer() < distance) {
                moveAtPower(power);
            } else {
                moveAtPower(0.0);
                return 0;
            }
        }
        return 1;
    }

    public int waitByDistance(double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        } else if (power > 0) {
            if (getWheelOdometer() - wheelDistanceLandMark < distance) {

            } else {
                moveAtPower(0.0);
                return 0;
            }
        } else {
            if (wheelDistanceLandMark - getWheelOdometer() < distance) {

            } else {
                moveAtPower(0.0);
                return 0;
            }
        }
        return 1;
    }


    // positive power to turn left
    public void turnAtPower(double p) {

        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i].setPower(-p);
        }

        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i].setPower(p);
        }

    }

    public void turnAtPower(double left, double right) {

        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i].setPower(left);
        }


        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i].setPower(right);
        }
    }

    public int getWheelOdometer() {
        int d =0;
        for ( int i =0; i < leftMotors.length; i++) {
            d += leftMotors[i].getCurrentPosition();
        }
        for (int i =0; i < rightMotors.length; i++) {
            d += rightMotors[i].getCurrentPosition();
        }
        return d/(leftMotors.length+rightMotors.length);
    }

    public void getWheelLandmarks () {
        wheelDistanceLandMark = getWheelOdometer();
    }

    public void releaseGlyph () {

    }

    public void collectGlyph () {

    }

    public void stopGlyphWheels(){

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

    protected void setMovingPower (double pL, double pR) {

        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i].setPower(pL);
        }

        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i].setPower(pR);
        }
    }

    public static double getVuforiaLeftRightDistance(OpenGLMatrix pose ) {
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            //double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);
            double rDegree = rot.secondAngle;

            return tZ * Math.sin(rDegree) + tY * Math.cos(rDegree);
        } else {
            return 0.0;
        }
    }

    public static double getVuforiaFrontBackDistance ( OpenGLMatrix pose ) {
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            // double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);
            double rDegree = rot.secondAngle;

            return tZ * Math.cos(rDegree) - tY * Math.sin(rDegree);
        } else {
            return 0.0;
        }
    }

}
