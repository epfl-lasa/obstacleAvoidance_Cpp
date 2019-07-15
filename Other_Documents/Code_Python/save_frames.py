import cv2
import time
import os

def video_to_frames(input_loc, output_loc):
    """Function to extract frames from input video file
    and save them as separate frames in an output directory.
    Args:
        input_loc: Input video file.
        output_loc: Output directory to save the frames.
    Returns:
        None
    """
    try:
        os.mkdir(output_loc)
    except OSError:
        pass
    # Log the time
    time_start = time.time()
    # Start capturing the feed
    cap = cv2.VideoCapture(input_loc)
    # Find the number of frames
    video_length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1
    print ("Number of frames: ", video_length)
    count = 0
    flag_init = True
    T_prev = 0 
    print ("Converting video..\n")
    # Start converting the video
    while cap.isOpened():
        # Extract the frame
        ret, frame = cap.read()
        
        
        #print(cap.get(cv2.CAP_PROP_POS_MSEC)*0.001)
        
        if (cap.get(cv2.CAP_PROP_POS_MSEC)*0.001 - T_prev)>=2 or flag_init :
            flag_init = False
            # Write the results back to output location.
            cv2.imwrite(output_loc + "/%#05d.jpg" % (int(cap.get(cv2.CAP_PROP_POS_MSEC)*0.001)), frame)
            T_prev = cap.get(cv2.CAP_PROP_POS_MSEC)*0.001
            print(T_prev)
            
        count = count + 1
        # If there are no more frames left
        if (count > (video_length-1)):
            # Log the time again
            time_end = time.time()
            # Release the feed
            cap.release()
            # Print stats
            print ("Done extracting frames.\n%d frames extracted" % count)
            print ("It took %d seconds forconversion." % (time_end-time_start))
            break

import cv2
import glob

def crop_images():
    #for name in glob.glob("I:/Videos_Experiments/Normal_VS_Bezier/*.png"):
    for name in glob.glob("/home/leziart/Documents/Metrics_Result/*.png"):
        img = cv2.imread(name)
        margin_x = 320
        off_x = 0
        margin_y = 100
        off_y = 25
        crop_img = img[(off_y+margin_y):(-margin_y+off_y),(off_x+margin_x):(-margin_x+off_x)]
        #crop_img_re = cv2.resize(crop_img, (640, 480))
        #cv2.imshow("cropped", crop_img_re)
        #print("I:/Videos_Experiments/Normal_VS_Bezier/Cropped/" + name[39:])
        #cv2.imwrite("I:/Videos_Experiments/Normal_VS_Bezier/Cropped/" + name[39:], crop_img)
        cv2.imwrite("/home/leziart/Documents/Metrics_Result/Cropped/" + name[39:], crop_img)
        #cv2.waitKey(0)
    print("### FINISHED ###")
        
if __name__=="__main__":

    # input_loc = 'video1.mp4'
    # output_loc = 'I:/Videos_Experiments/Frames/video1'
    # video_to_frames(input_loc, output_loc)
    crop_images()
    


