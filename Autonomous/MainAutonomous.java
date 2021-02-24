// autonomous main file 
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
/*import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;*/

import java.util.List;

//Autonomous

@Autonomous(name = "Autonomus", group = "LinearOpMode")
public class Auto_Code extends LinearOpMode {

    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor Shooter1;
    public PIDController pidRotate;
    public Orientation lastAngles = new Orientation();
    double globalAngle,rotation;
    double start_time;
    BNO055IMU imu;


    public void Forward(double rotations, double power) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition((int) (1120 * rotations));
        FrontRight.setTargetPosition((int) (1120 * rotations));
        BackLeft.setTargetPosition((int) (1120 * rotations));
        BackRight.setTargetPosition((int) (1120 * rotations));

        waitForStart();

        FrontLeft.setPower(1 * power);
        FrontRight.setPower(1 * power);
        BackLeft.setPower(1 * power);
        BackRight.setPower(1 * power);

        start_time = getRuntime();

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontLeft.isBusy() && FrontRight.isBusy()) {
            telemetry.update();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }


    public void Backward(double rotations, double power) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition((int) (-1120 * rotations));
        FrontRight.setTargetPosition((int) (-1120 * rotations));
        BackLeft.setTargetPosition((int) (-1120 * rotations));
        BackRight.setTargetPosition((int) (-1120 * rotations));

        waitForStart();

        FrontLeft.setPower(-1 * power);
        FrontRight.setPower(-1 * power);
        BackLeft.setPower(-1 * power);
        BackRight.setPower(-1 * power);

        start_time = getRuntime();

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontLeft.isBusy() && FrontRight.isBusy()) {
            telemetry.update();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }


    public void Left_point_Turn(double rotations, double power) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition((int) (-1120 * rotations));
        FrontRight.setTargetPosition((int) (1120 * rotations));
        BackLeft.setTargetPosition((int) (-1120 * rotations));
        BackRight.setTargetPosition((int) (1120 * rotations));

        waitForStart();

        FrontLeft.setPower(-1 * power);
        FrontRight.setPower(1 * power);
        BackLeft.setPower(-1 * power);
        BackRight.setPower(1 * power);

        start_time = getRuntime();

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontLeft.isBusy() && FrontRight.isBusy()) {
            telemetry.update();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }


    public void Right_point_Turn(double rotations, double power) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition((int) (1120 * rotations));
        FrontRight.setTargetPosition((int) (-1120 * rotations));
        BackLeft.setTargetPosition((int) (1120 * rotations));
        BackRight.setTargetPosition((int) (-1120 * rotations));

        waitForStart();

        FrontLeft.setPower(1 * power);
        FrontRight.setPower(-1 * power);
        BackLeft.setPower(1 * power);
        BackRight.setPower(-1 * power);

        start_time = getRuntime();

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontLeft.isBusy() && FrontRight.isBusy()) {
            telemetry.update();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    private void strafeLeft(double rotations,double power){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition((int) (-1120 * rotations));
        FrontRight.setTargetPosition((int) (1120 * rotations));
        BackLeft.setTargetPosition((int) (1120 * rotations));
        BackRight.setTargetPosition((int) (-1120 * rotations));

        waitForStart();

        FrontLeft.setPower(-1 * power);
        FrontRight.setPower(1 * power);
        BackLeft.setPower(1 * power);
        BackRight.setPower(-1 * power);

        start_time = getRuntime();

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontLeft.isBusy() && FrontRight.isBusy()) {
            telemetry.update();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }

    private void strafeRight(double rotations,double power){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition((int) (1120 * rotations));
        FrontRight.setTargetPosition((int) (-1120 * rotations));
        BackLeft.setTargetPosition((int) (-1120 * rotations));
        BackRight.setTargetPosition((int) (1120 * rotations));

        waitForStart();

        FrontLeft.setPower(1 * power);
        FrontRight.setPower(-1 * power);
        BackLeft.setPower(-1 * power);
        BackRight.setPower(1 * power);

        start_time = getRuntime();

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontLeft.isBusy() && FrontRight.isBusy()) {
            telemetry.update();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }

    private void Shooter(double power) {
        Shooter1.setPower(power);
    }

    private void setMovement(double lx, double ly, double rx, double power){
        FrontLeft.setPower(Range.clip(ly + lx + rx, -power, power));
        FrontRight.setPower(Range.clip(ly - lx - rx, -power, power));
        BackRight.setPower(Range.clip(ly - lx + rx, -power, power));
        BackRight.setPower(Range.clip(ly + lx - rx, -power, power));

    }

    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                FrontLeft.setPower(power);
                FrontRight.setPower(-power);
                BackLeft.setPower(power);
                BackRight.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                FrontLeft.setPower(-power);
                FrontRight.setPower(power);
                BackLeft.setPower(-power);
                BackRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                FrontLeft.setPower(-power);
                FrontRight.setPower(power);
                BackLeft.setPower(-power);
                BackRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public void Position_A() {

        Forward(4, 0.5);
        sleep(1000);
        Right_point_Turn(1, 0.5);
        sleep(1000);
        Forward(1, 0.5);
        sleep(1000);
        Backward(1, 0.5);
        Left_point_Turn(1, 0.5);
        sleep(1000);
        Backward(1, 0.5);
    }


    public void Position_B() {
        Forward(5.5, 0.5);
        sleep(1000);
        Backward(3, 0.5);

    }


    public void Position_C() {

        Forward(7.8, 0.5);
        sleep(1000);
        Right_point_Turn(1, 0.5);
        sleep(1000);
        Forward(0.8, 0.5);
        sleep(1000);
        Backward(0.8, 0.5);
        Left_point_Turn(1, 0.5);
        sleep(1000);
        Backward(5, 0.5);
    }


    /*private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AZL+ESX/////AAAAmSIWp50Ri0AenYSCRXT9VwQ+S1Pw1+TKNGgPV5KpleBAz4bkXUMZ46PGPcpNGX/RT1bhl8AbtbltsRTQejJSl0K5JurINnhGk9kW9ezGsHWmHI9NU8Nqd/JP2RRJWRtD6x++T83w6fMvl5acLbseV3PulaaWAzQQ7Xvnnqm4ovoKAbWLojVH485fjkijXUgVXLCaszwdrNEZFrEmdoiwdMnPiEYNSyNrlkO/ZZ2m0wLrevDQP9beFWsZ8Aqw5tfFDEJ1OYmUEqijUcsqPDsA0v0yXMuKRQGs8SLIQG0pD3F09qzwuWOsowf+JP6DGTo9RdOlwsESw/QFZA7g47mCHAoTuB7KlXqJuMH4gRye8gqT";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);*/
    }



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override

    public void runOpMode() {
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        Shooter1 = hardwareMap.dcMotor.get("Shooter1");
        pidRotate = new PIDController(2,3,4);
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //initVuforia();
        //initTfod();
        String Ring = null;



        // Put initialization blocks here.

        /*if (tfod != null) {
            tfod.activate();
        }
        telemetry.addData(">", "Starting camera of Robot Controller. Please wait.");
        telemetry.update();
        sleep(2000);*/
        telemetry.addData(">", "Click on Play to start op mode");
        telemetry.update();
        sleep(2000);

 /*       List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());

            int i = 0;
            for (Recognition recognition : updatedRecognitions)
            {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                Ring=recognition.getLabel();
            }
*/


        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int i = 0;
                        for (Recognition recognition : updatedRecognitions)
                        {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                            Ring=recognition.getLabel();
                        }
                        telemetry.update();
                    }

                    if(Ring=="Quad")
                    {
                        Position_C();
                    }
                    else if(Ring=="Single")
                    {
                        Position_B();
                    }
                    else
                    {
                        Position_A();
                    }
                    break;

                }
            }

            if (tfod != null)
            {
                tfod.shutdown();
            }
        }
    }
}
