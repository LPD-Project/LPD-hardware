import argparse
from DarknetObjectDetector import *
import socketio

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default="/dev/video0",help="video source. If empty, uses webcam 0 stream")
                        #gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! xvimagesink sync=false
    parser.add_argument("--out_filename", type=str, default="",
                        help="inference video name. Not saved if empty")
    parser.add_argument("--weights", default="/home/pigeon/darknet/LPD-imageDetection/yoloModel-temp/yolov4-tiny-1/v6/custom-yolov7-tiny-detector_best.weights", #v6/custom-yolov7-tiny-detector_best.weights #v5/custom-yolov4-tiny-detector_last.weights 
                        help="yolo weights path")
    parser.add_argument("--dont_show", action="store_true",
                        help="window inference display. For headless systems")
    parser.add_argument("--ext_output", action="store_true",
                        help="display bbox coordinates of detected objects")
    parser.add_argument("--config_file", default="/home/pigeon/darknet/LPD-imageDetection/yoloModel-temp/yolov4-tiny-1/v6/custom-yolov7-tiny-detector.cfg",      #v6/custom-yolov7-tiny-detector.cfg       #v5/custom-yolov4-tiny-detector.cfg
                        help="path to config file")
    parser.add_argument("--data_file", default="/home/pigeon/darknet/LPD-imageDetection/yoloModel-temp/yolov4-tiny-1/v6/obj.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")
    return parser.parse_args()

if __name__ == "__main__":
    args = parser()

    # Create ObjectDetector
    darknet_detector = DarknetObjectDetector(
        input_path=args.input,
        weights=args.weights,
        config_file=args.config_file,
        data_file=args.data_file,
        thresh=args.thresh,
        out_filename=args.out_filename
    )
    
    # Create a Socket.IO client instance
    try:
        sio = socketio.Client()
        sio.connect('http://192.168.50.10:3000')
      
        laserState = None
        cameraState = None

        # Define event handlers
        @sio.event
        def connect():
            print('Connected to raspi socket server')
    
        @sio.event
        def disconnect():
            print('Disconnected from server')
     
        @sio.event
        def jetsonLaserCommand(data):
            print('Received OnLaserControl event:', data)
            global laserState
            laserState = data["laser"]
            print("socket start laser ")
	    
            if laserState == "false":
                darknet_detector.turnLaserOff()
            else:
                darknet_detector.turnLaserOn()


        @sio.event
        def jetsonCameraCommand(data):
            print('Received OnCameraControl event:', data)
            global cameraState
            cameraState = data["camera"]
            print("socket start camera ")
            if cameraState == "false":
                # Stop camera(model)
                darknet_detector.stop()
                darknet_detector.turnLaserOff()
                # stop laser 

            else:
                darknet_detector.restart()

        # Connect to the server

        # Wait for events

        # Start ObjectDetector
        darknet_detector.run()
        # When you want to stop the detection process
        # darknet_detector.stop()

        # When you want to restart the detection process
        # darknet_detector.restart()
    except:
        darknet_detector.run()
