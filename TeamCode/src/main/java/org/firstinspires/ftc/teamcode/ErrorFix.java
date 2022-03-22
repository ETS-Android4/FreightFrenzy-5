package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp

public class ErrorFix extends LinearOpMode 
{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu;
    private Chassis chassis = null;

    @Override
    public void runOpMode() 
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        chassis = new Chassis( leftDrive, rightDrive, imu, telemetry ); 
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) 
        {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double angle = 0.0;
            if (Math.abs(y) >= Math.abs(x)) {
                if (y <= 0)
                    angle = 0.0;
                else
                    angle = 180;
            }
            else {
                if(x > 0) 
                    angle = 90;
                else
                    angle = -90;
            }
            
            if( gamepad1.a ) {
                chassis.turn(angle, 1.0, 1.0);
                sleep(2000);
            }

            telemetry.addData("Angle", "%f", angle);
            telemetry.update();
        }
    }
}
