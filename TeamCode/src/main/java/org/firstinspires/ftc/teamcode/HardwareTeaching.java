package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.DelayQueue;

/**

 24 inches have 2082 (1939) encoder counts that is 86.75(80.79)encoder counts per inch
 Nathan has wheel base of (15.25) inches.
 */
public class HardwareTeaching extends HardwareBase
{
    // DC Motors
    public DcMotor motorLeftWheel = null;
    public DcMotor motorRightWheel = null;

    ModernRoboticsI2cGyro gyro = null;

    /* Constructor */
    public HardwareTeaching(){

    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        motorLeftWheel = hwMap.dcMotor.get("leftWheel");
        motorRightWheel = hwMap.dcMotor.get("rightWheel");
        motorLeftWheel.setDirection(DcMotor.Direction.REVERSE);  // 40 to 1 andymark motor
        motorRightWheel.setDirection(DcMotor.Direction.FORWARD); // 40 to 1 andymark motor
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

    }

    @Override
    public void start() {
        // wheels
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);
    }

    @Override
    public void stop() {
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);

        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public float getGyroHeading () {
        return -gyro.getHeading();
    }
}