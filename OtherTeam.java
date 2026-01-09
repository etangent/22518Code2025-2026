

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import 22518Code2025-2026.Tuner;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static java.lang.Math.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

@TeleOp(name = "odometryTest (Java)")
public class OtherTeam extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    
    
    private Pose2D pos;
    private Pose2D vel;
    private Pose2D alignPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    
    Tuner tuner = new Tuner(
      new Tuner.Param("strafeRatio", 1.2, .001),
      new Tuner.Param("driveKp", .005, .001),
      new Tuner.Param("driveKd", .001, .001),
      new Tuner.Param("turnKp", 8, .001),
      new Tuner.Param("turnKd", .6, .001)  
    );
    
    double x;
    double y;
    double rx;

    // just logging align values
    double Vx = 0;
    double Vy = 0;
    
    SparkFunOTOS odo;
    
    double time = 0;
    
    
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(SparkFunOTOS.class, "odo");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        
        
        odo.setLinearUnit(DistanceUnit.MM);
        odo.setAngularUnit(AngleUnit.RADIANS);
        
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        odo.setOffset(offset);

        odo.setLinearScalar(1.0);
        odo.setAngularScalar(1.0);
        
        odo.calibrateImu();
        odo.resetTracking();

        // Use if you don't want the robot to start at the origin. You make somewhere else 0, 0, 0.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odo.setPosition(currentPosition);

        // Get the hardware and firmware version (remember to set these!)
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        odo.getVersionInfo(hwVersion, fwVersion);
        
        waitForStart();
        resetRuntime();
        
        while (opModeIsActive()) {
            pos = odo.getPosition();
            vel = odo.getVelocity();    
            tuner.update(gamepad1);
            
            if (gamepad1.b) {
                odo.calibrateImu();
                odo.resetTracking();
            } else if (!gamepad1.a) {
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
                drive();
            } else {
                align(alignPose);
            }
            
            double newTime = getRuntime();
            telemetry.addData("Loop time", newTime - time);
            time = newTime;
            

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.x, pos.y, pos.h);
            telemetry.addData("Position", data);
            telemetry.addLine("\n=== Tuner ===");
            for (Tuner.Param param : tuner.getParams()) {
                if (param == tuner.getSelected())
                    telemetry.addData("> " + param.name, "%.4f  (inc=%.4f)", param.value, param.increment);
                else
                    telemetry.addData("  " + param.name, "%.4f", param.value);
            }
            String vector = String.format(Locale.US, "{Vx: %.3f, Vy: %.3f}", Vx, Vy);
            telemetry.addData("Displacement vector", vector);
            telemetry.update();
        }
    }

    public void align(Pose2D target) {
        double heading = pos.h;
        // field relative: x is forward, y is to the right
        double yError = (target.y - pos.y);
        double xError = (target.x - pos.x);
        double turnError = AngleUnit.normalizeRadians(target.h - heading);

        double distance = hypot(xError, yError);
        double errorVel = (xError * vel.x + yError * vel.y) / distance;

        x = 0;
        y = 0;
        rx = 0;
        
        if (!atHeading(target.h)) {
            rx = -1 * tuner.get("turnKp") * turnError + tuner.get("turnKd") * vel.h;
        } else if (distance > 10) {
            double setpointScale = Math.min(tuner.get("driveKp") * distance - tuner.get("driveKd") * errorVel, 1 / tuner.get("strafeRatio"));
            Vx = xError / distance * setpointScale;
            Vy = yError / distance * setpointScale;
            // now converting to robot relative, y is forward
            y = Vx * cos(heading) + Vy * sin(heading);
            x = -1 * tuner.get("strafeRatio") * (-Vx * sin(heading) + Vy * cos(heading));
        }
        
        drive();
    }
    
    private boolean atHeading(double heading) {
        return Math.abs(AngleUnit.normalizeRadians(heading - pos.h)) < 0.1;
    }
    
    private void drive() {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FrontLeft.setPower((y + x + rx)/denominator);
        FrontRight.setPower(((y - x) - rx)/denominator);
        BackLeft.setPower(((y - x) + rx)/denominator);
        BackRight.setPower(((y + x) - rx)/denominator);
    }
}
