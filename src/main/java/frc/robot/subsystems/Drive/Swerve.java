package frc.robot.subsystems.Drive;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.util.Units;

public class Swerve extends SubsystemBase{
    double maxspeed = Units.feetToMeters(2.0);
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(), "swervedrive");
    public Swerve(File directory){
        try {
            SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(maxspeed);
   
        } catch (Exception e) {
            // TODO: handle exception
        }

    }
}
