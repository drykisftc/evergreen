package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 24 inches have 1939 encoder counts that is 80.79encoder counts per inch
 Nathan has wheel base of 14.4 inches.
 */

public class HardwareLift extends HardwareBase
{
    // DC Motors
    public DcMotor liftMotor = null;

    /* Constructor */
    public HardwareLift(){

    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);


    }

    @Override
    public void start () {

        
    }

    @Override
    public void stop() {


        
    }


}
