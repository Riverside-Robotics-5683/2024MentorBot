package ravenrobotics.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    // TODO: Change cameraName to the name of your camera
    private final PhotonCamera camera = new PhotonCamera("photonvision");
        // Query the latest result from PhotonVision
    private PhotonTrackedTarget target;
    private double range, yaw, cameraHeight, cameraPitch, targetPitch, targetHeight, goalRange;
    private int targetID;

    //TODO: Update vision Constants
    public VisionSubsystem(double tHeight, double gRange){
        cameraHeight = Constants.VisionConstants.kCameraHeight;
        cameraPitch = Constants.VisionConstants.kCameraPitchAngle;
        targetHeight = tHeight;//height of target
        goalRange = gRange;//how close to target
     } 
    
    @Override
    public void periodic() {
        updateTargetData();
        System.out.println("Yaw: " + yaw + ", Target ID: " + targetID + " , Distance to target: " + range);
    }

    // Get information from target.
    public void updateTargetData(){

        // Update the target data in the periodic block
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            
            // Get the current best target.
            target = result.getBestTarget();
            yaw = target.getYaw();//The yaw of the target in degrees (positive right).
            targetPitch = target.getPitch();//The pitch of the target in degrees (positive up).
            targetID = target.getFiducialId();//The ID of the detected fiducial marker.
            
            //Distance to target
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight, 
                    targetHeight,
                    cameraPitch,
                    Units.degreesToRadians(targetPitch));
        }

    }

    //return most recent yaw
    public double getTargetYaw(){
        return yaw;
    }

    //return most recent distance to target
    public double getTargetDistance(){
        return range;
    }

    //return most recent target ID
    public int getTargetID(){
        return targetID;
    }
    
    public double getGoalRange(){
        return goalRange;
    }

    //TODO: for testing map to button press. Check out how this works.
    /* 
    Request the camera to save a new image file from the output stream with overlays. 
    Images take up space in the filesystem of the PhotonCamera. 
    Calling it frequently will fill up disk space and eventually 
    cause the system to stop working. 
    Clear out images in /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues. 
    */
    public void getCameraImage(){
        // Capture pre-process camera stream image
        // camera.takeInputSnapshot();

        // Capture post-process camera stream image
        camera.takeOutputSnapshot();
    }
}