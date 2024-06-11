package frc.robot.subsystems.Drive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.sim.ChassisReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.AutonConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Swerve extends SubsystemBase{
    double maxspeed = Units.feetToMeters(2.0);
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(), "swervedrive");
    SwerveDrive swerveDrive;
   
    public Swerve(File directory){
        //turn off swerve drive telemetry
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(maxspeed);
   
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
       swerveDrive.setHeadingCorrection(false);
       swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);


    }
    
    public void SwerveSubsystem(SwerveDriveConfiguration driveCFG, SwerveControllerConfiguration controlCFG){
        swerveDrive = new SwerveDrive(driveCFG, controlCFG, maxspeed);
    }
    public void setUpPathPlanner() {
        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         AutonConstants.TRANSLATION_PID,
                                         // Translation PID constants
                                         AutonConstants.ANGLE_PID,
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
    }
    //use for pathfinding during auto
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(swerveDrive.getMaximumVelocity(), 
                                    4.0, swerveDrive.getMaximumAngularVelocity(), 
                                    Units.degreesToRadians(720));
        
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0);
    }


    //Drive command where rotational input determines robot heading
    // public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
    //                             DoubleSupplier rotationX, DoubleSupplier rotationY){
    //     swerveDrive.setHeadingCorrection(true);
    //     return run(()-> {
    //         double xInput = Math.pow(translationX.getAsDouble(), 3);
    //         double yInput = Math.pow(translationY.getAsDouble(), 3);
    //         driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 
    //                                                         rotationX.getAsDouble(), 
    //                                                         rotationY.getAsDouble(), 
    //                                                         swerveDrive.getOdometryHeading().getRadians(),
    //                                                         swerveDrive.getMaximumVelocity()));
    //     });
    // }

    //drive command that uses rotational input as speed of rotation
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                                DoubleSupplier rotationX){
        return run(() -> {
            swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                                Math.pow(rotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                                true, false);
        });
    }
    public void driveFieldOriented(ChassisSpeeds velocity){
        swerveDrive.driveFieldOriented(velocity);
    }

    public SwerveDriveKinematics getKinematics(){
        return swerveDrive.kinematics;
    }
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }
}
