package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    public static Command testAuto() throws FileVersionException, IOException, ParseException {
        PathPlannerPath testPath = PathPlannerPath.fromChoreoTrajectory("TestPath");
        Command followPath = AutoBuilder.followPath(testPath);
        Command resetOdom = AutoBuilder.resetOdom(testPath.getStartingHolonomicPose().orElse(new Pose2d()));
        return
            resetOdom.andThen(
            followPath);
    }
}