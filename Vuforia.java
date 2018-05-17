package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name="Vuforia", group ="Autonomous")
public class vuforia extends LinearOpMode {

  //public static final String TAG = "Vuforia VuMark Sample";

  OpenGLMatrix lastLocation = null;

  VuforiaLocalizer vuforia;

  @Override public void runOpMode() {

    int time = 0;
    int valuesToCollect = 100; // Must be even
    double xAltered;
    double zAltered;
    double xMedian = 0;
    double zMedian = 0;
    double xAvg = 0;
    double zAvg = 0;
    double xValues[] = new double[valuesToCollect];
    double zValues[] = new double[valuesToCollect];
    double xFinal = 0;
    double zFinal = 0;

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    parameters.vuforiaLicenseKey = "INSERT VUFORIA LICENSE KEY HERE";

    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);

    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    waitForStart();

    relicTrackables.activate();

    while (opModeIsActive()) {
      RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
      if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
        telemetry.addData("VuMark", "%s visible", vuMark);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
        telemetry.addData("Pose:", pose);

        if (pose != null) {
          VectorF position = pose.getTranslation();
          Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

          double xPos = position.get(0);
          double yPos = position.get(1);
          double zPos = position.get(2);

          double xRot = rot.firstAngle;
          double yRot = rot.secondAngle;
          double zRot = rot.thirdAngle;

          xAltered = ((yRot * (zPos - 15)) - (60 * xPos))/540;
          zAltered = zPos/9;
          
          if(time < valuesToCollect){
            xValues[time] = xAltered;
            zValues[time] = zAltered;
            time ++;
          }else if(time >= valuesToCollect){
            Arrays.sort(xValues);
            Arrays.sort(zValues);
            xMedian = Math.round(xValues[valuesToCollect/2]*1000);
            xMedian = xMedian/1000;
            zMedian = Math.round(zValues[valuesToCollect/2]*1000);
            zMedian = zMedian/1000;
            xAvg = 0;
            zAvg = 0;
            for(int i = 0; i < xValues.length; i ++){
              xAvg += xValues[i];
            }
            for(int i = 0; i < zValues.length; i ++){
              zAvg += zValues[i];
            }
            xAvg = xAvg/xValues.length;
            xAvg = Math.round(xAvg * 1000);
            xAvg = xAvg/1000;
            zAvg = zAvg/zValues.length;
            zAvg = Math.round(zAvg * 1000);
            zAvg = zAvg/1000;
            time = 0;
            xFinal = (xAvg + xMedian)/2;
            zFinal = (zAvg + zMedian)/2;
          }
          telemetry.addData("FinalX:", xFinal);
          telemetry.addData("FinalZ:", zFinal);
        }
      }else{
        telemetry.addData("VuMark", "not visible");
      }
      telemetry.update();
    }
  }
}
