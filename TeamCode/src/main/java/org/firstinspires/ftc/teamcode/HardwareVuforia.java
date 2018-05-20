package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

/**
 * This is NOT an OpMode
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot, using Matrix Hardware.
 * See PushbotTeleopTank_Iterative for a usage examples.
 *
 * This is coded as an Extension of HardwarePushbot to illustrate that the only additional
 * action REQUIRED for a MATRIX controller is enabling the Servos.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Matrix Controller has been assigned the name:  "matrix controller"
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 *
 * In addition, the Matrix Controller has been assigned the name:  "matrix controller"
 */
public class HardwareVuforia extends HardwareBase
{
    //Vuforia
    VuforiaLocalizer vuforia;

    protected VuforiaLocalizer.CameraDirection cameraDirection;
    protected int cameraMonitorViewId;
    protected VuforiaLocalizer.Parameters parameters;
    protected VuforiaTrackables relicTrackables;
    protected VuforiaTrackable relicTemplate;
     RelicRecoveryVuMark vuMark;
    String vumarkImage = "unknown";

    /* Constructor */
    public HardwareVuforia(VuforiaLocalizer.CameraDirection cameraD){
        cameraDirection = cameraD;
    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);
        // camera
        cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVw9AA7/////AAAAGR2dOk5hfEdLl+V9Doao7C5Xp0Wvb2cien7ybTAhAyUTB2iZRO/CMyxlXakNnP3+HqLEMe7nzV+fllHLVQLuSwWmLdDErkjexTZKcgCGQUIZ+Ts6O2m7l+zwVVBH5V5Ah5SJP3jd/P6lvuKJY+DUY0pThAitsP59uD6wkcukMQQXNN+xBPzEBEx/0kt7hS5GJ+qCYDLD1qgCO5KrDuWzYtWjZi3LaGHsO9msvrGiCXYaP9PDRX9ZoWB1tJiHky5HyG/p+ndycmiK6sY9lRymaaJ5fX556ZUKtQX2dOAF7tHgVqsPOhqCV3E3qN6kXnwEqy9KgZ1QQjKJnCR5eLRXmSOqAbKi8ArzrRc3737EpSzK";
        parameters.cameraDirection = cameraDirection;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

    }

    @Override
    public void start() {
        relicTrackables.activate();
    }

    @Override
    public void stop() {
        relicTrackables.deactivate();
    }

    public void identifyGlyphCrypto (){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            vumarkImage = "left";
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            vumarkImage = "right";
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            vumarkImage = "center";
        }
    }

    public OpenGLMatrix getGlyphCryptoPosition() {
        return ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
    }

    public Image getImage(){
        VuforiaLocalizer.CloseableFrame frame = null;
        try{
            frame = vuforia.getFrameQueue().take();
            long numImages = frame.getNumImages();
            Image rgbImage = null;
            for (int i = 0; i < numImages; i++) {
                Image img = frame.getImage(i);
                int fmt = img.getFormat();
                if (fmt == PIXEL_FORMAT.RGB565) {
                    rgbImage = frame.getImage(i);
                    break;
                }
            }
            return rgbImage;
        }
        catch(InterruptedException exc){
            return null;
        }
        finally{
            if (frame != null) frame.close();
        }
    }

    public Mat vuforiaImageToOpenCvImage (Image rgb) {

        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        //put the image into a MAT for OpenCV
        Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);

        return tmp;
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