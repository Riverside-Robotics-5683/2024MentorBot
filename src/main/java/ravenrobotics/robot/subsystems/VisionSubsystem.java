package ravenrobotics.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    // TODO: Change cameraName to the name of your camera
    private final PhotonCamera camera = new PhotonCamera("photonvision");
        // Query the latest result from PhotonVision
    private PhotonTrackedTarget target;
    private double yaw;
 
    
    public VisionSubsystem(){
        //TODO: Any required set up for camera?
    } 
      
    public void getTargetData(){
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            // Get the current best target.
            target = result.getBestTarget();
            yaw = target.getYaw();//The yaw of the target in degrees (positive right).
        }
    }

    // Get information from target.
    public double getTargetYaw(){
        return yaw;
    }

}
