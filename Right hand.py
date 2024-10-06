import cv2
import numpy as np
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

def calculate_angle(a,b,c):
    a = np.array(a) # First
    b = np.array(b) # Mid
    c = np.array(c) # End
    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)
    
    if angle >180.0:
        angle = 360-angle
        
    return angle 





cap = cv2.VideoCapture("sample.mp4")






# Curl counter variables
sqcounter = 0 
pushcounter=0

stage = None


## Setup mediapipe instance
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        ret, frame = cap.read()
        
        
        # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
      
        # Make detection
        results = pose.process(image)

    
        # Recolor back to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        image = cv2.resize(frame, (1920//2,1080//2))
        
        # Extract landmarks
        try:
            landmarks = results.pose_landmarks.landmark
            
           
            
            # Get right coordinates for squats
            rhip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
            rknee = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
            rankle = [landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]
            
            # Get left coordinates for squats

            lhip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
            lknee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x,landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
            lankle = [landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x,landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y]
            
           
           # Get coordinates for pushup left leg
            lfoot = [landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].x,landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].y]
            
            lshoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
            lelbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            lwrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            

            # Calculate angle FOR SQUATS 
            sqlangle = calculate_angle(rhip, rknee, rankle)
            sqrangle = calculate_angle(lhip, lknee, lankle)

            # Calculate angle for Pushups

            pushLangle=calculate_angle(lknee, lankle, lfoot)

            pushangle=calculate_angle(lshoulder,lelbow,lwrist)

            

          


            
            # Visualize angle for squats
            
            cv2.putText(image, str(pushangle), 
                           tuple(np.multiply(lelbow, [640, 480]).astype(int)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                )
            # Curl counter logic
            
            if pushLangle > 60 and pushLangle <110:
                position="Push ups"
                if pushangle < 110: 

                    stage = "Down"
                if pushangle > 110  and stage =='Down':
                    stage="Up"
                    pushcounter +=1
            else:
                position="Squat"

           
                
                if sqrangle <= 175 and sqlangle <=175:
                    stage = "Down"
                if sqrangle > 178 and sqlangle >178 and stage =='Down':
                    stage="Up"
                    sqcounter +=1
            
            
            
               
                
                       
        except:
            pass
        
        # Render curl counter
        # Setup status box
        cv2.rectangle(image, (0,0), (300,250), (255,255,255), -1)
        
        # Rep data
       
        
        # Stage data
        cv2.putText(image, position, (0,70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 2, cv2.LINE_AA)
        cv2.putText(image, str(stage), 
                    (0,140), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 2, cv2.LINE_AA)
        
        
        if position=="Push ups":
            cv2.putText(image, str(pushcounter), 
                    (0,210), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 6, cv2.LINE_AA)
        else:
            cv2.putText(image, str(sqcounter), 
                    (0,210), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 2, cv2.LINE_AA)
        
        # Render detections
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                mp_drawing.DrawingSpec(color=(66,206,245), thickness=2, circle_radius=2), 
                                mp_drawing.DrawingSpec(color=(245,206,52), thickness=2, circle_radius=2) 
                                 )               
        
        cv2.imshow('Curl counter', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()








    







