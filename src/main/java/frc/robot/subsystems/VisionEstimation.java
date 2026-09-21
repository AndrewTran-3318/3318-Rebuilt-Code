 package frc.robot.subsystems;

 import java.util.List;
 import java.util.Optional;

 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.targeting.PhotonTrackedTarget;

 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import frc.robot.Constants;
 import frc.robot.RobotContainer;




//see https:github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
 public class VisionEstimation{
     private PhotonCamera camera;
     private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(Constants.APRILTAGLAYOUT, Constants.CAMERAPOSITION);
     private Matrix<N3, N1> curStdDevs;
     public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
     public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

     public VisionEstimation(PhotonCamera camera)
     {
        this.camera = camera;
     }


     public void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = kSingleTagStdDevs;

         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;

             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }

             if (numTags == 0) {
                  //No tags visible. Default to single-tag std devs
                 curStdDevs = kSingleTagStdDevs;
             } else {
                  //One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                  //Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                  //Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }

     public Optional<EstimatedRobotPose> visionEstimationUpdate()
     {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var result : camera.getAllUnreadResults()) {
             visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
             if (visionEst.isEmpty()) {
                 visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
             }
             updateEstimationStdDevs(visionEst, result.getTargets());


             visionEst.ifPresent(
                     est -> {
                          //Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                     });
         }
         return visionEst;
     }

     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
     
 }
