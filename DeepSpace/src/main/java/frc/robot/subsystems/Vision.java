package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import java.util.Optional;

import javax.print.attribute.standard.Media;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Vision;


public class Vision extends SubsystemBase { 

    private Vision visionInterface = new Vision();
    private PhotonPipelineResult noteResult = visionInterface.getLatestNoteResult();

}
