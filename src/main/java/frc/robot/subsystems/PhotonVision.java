package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class PhotonVision {

    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonTrackedTarget target;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d robotToCam;
    private boolean hasTargets;
    private double targetYaw, targetPitch, targetArea, targetSkew, poseAmbiguity;

    private List<PhotonTrackedTarget> targets;
    private Transform2d targetPose;
    private List<TargetCorner> targetCorners;
    private Transform3d bestCameraToTarget, alternateCameraToTarget;
    private PIDController translationPID, rotationPID;

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0); //CHANGE BASED ON CAMERA PLACEMENT
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(21.7);
    

    public PhotonVision(String string) {
        

        camera = new PhotonCamera("camera");
        camera.setDriverMode(true);
        
        robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
        camera.setPipelineIndex(2);

        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        var result = camera.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d fieldtoCamera = result.getMultiTagResult().estimatedPose.best;
        }
        
        target = result.getBestTarget();

        hasTargets = result.hasTargets();

        targets = result.getTargets();

        targetYaw = target.getYaw();
        targetPitch = target.getPitch();
        targetArea = target.getArea();
        targetSkew = target.getSkew();
       // targetPose = target.getCameraToTarget();
        targetCorners = target.getDetectedCorners();

        int targetID = target.getFiducialId();
        poseAmbiguity = target.getPoseAmbiguity();
        bestCameraToTarget = target.getBestCameraToTarget();
        alternateCameraToTarget = target.getAlternateCameraToTarget();

        translationPID = new PIDController(0.1, 0, 0); //tune values
        rotationPID = new PIDController(0.1, 0, 0); //tune values

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
