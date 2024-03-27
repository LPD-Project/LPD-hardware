import darknet
import cv2
import time
import threading
import queue
import subprocess
from DetectedObjectList import*
from MapObject import*
from HardwareController import*

class DarknetObjectDetector:
    def __init__(self, input_path, weights, config_file, data_file, thresh=0.4, out_filename=""):
	
        self.input_path = input_path
        self.weights = weights
        self.config_file = config_file
        self.data_file = data_file
        self.thresh = thresh
        self.out_filename = out_filename
        self.stop_flag = threading.Event()
        self.detected_objects = DetectedObjectList()  # สร้าง DetectedObjectList
        self.hw_ctl = HardwareController(640, 480, 30, 18, 48, 30) #1920 #1080


    @staticmethod
    def restart_nvargus_daemon():
    	try:
            # เรียกใช้ subprocess เพื่อรันคำสั่ง sudo service nvargus-daemon restart
            # subprocess.run([ 'service', 'nvargus-daemon', 'restart'], check=True)
            print("Restarted nvargus-daemon service successfully.")
    	except subprocess.CalledProcessError as e:
            print("Error restarting nvargus-daemon service:", e)
    
    def stop(self):#****************************************************************
        """Stop the object detection process."""
        pass
        #self.stop_flag.set()  # Set the stop flag to stop the current process

    def restart(self):#****************************************************************
        """Restart the object detection process."""
        self.stop_flag.set()  # Set the stop flag to stop the current process
        self.stop_flag.clear()  # Clear the flag for the next run
        # Re-initialize any necessary variables or resources
        self.detected_objects.clear()  # Clear detected objects from previous runs
        # Start the object detection process again
        threading.Thread(target=self.run).start()
    
    def turnLaserOff(self):
        self.hw_ctl.laserIsOn = False

    def turnLaserOn(self):
        self.hw_ctl.laserIsOn = True

    def getLaserIsOn(self):
        return self.hw_ctl.laserIsOn
   
    def to_another(self,detected_objects):
        # ทำงานที่ต้องการทำกับ detected_objects ที่ได้รับเข้ามา***********************************************************************************************
        #calibration = True
        person_data = []
        pigeon_data = []
        pigeon_pos = []
        #self.hw_ctl.laserIsOn = True
      
        
        if self.hw_ctl.laserIsOn == False:
            self.hw_ctl.laser_off()
            return
        if detected_objects == []:
            #print("detected object == []")
            self.hw_ctl.laser_off()
            self.hw_ctl.moveServo(self.hw_ctl.init_position)
            return
        for item in detected_objects:
            print(item.left, item.right, item.top, item.bottom)
            if (item.class_name == "pigeon") &(float(item.confidence) >= 38.0):
                pigeon_data.append(item)
            elif (item.class_name == "person" )&(float(item.confidence) >= 40.0) :
                person_data.append(item)
        if pigeon_data != []:            # Check if pigeon_data is not empty
            if person_data != []: 
                self.hw_ctl.laser_on_25()
            else:
                self.hw_ctl.laser_on_100()
            for pi in pigeon_data:
                #pigeon_pos.append(((pi.left + pi.right) * 1920 / (2 * 512), (pi.top + pi.bottom) * 1080 / (2 * 512)))
                pigeon_pos.append(((pi.left + pi.right), (pi.top + pi.bottom)))
            sorted_pigeon = sorted(pigeon_pos, key=lambda p: self.hw_ctl.distance(self.hw_ctl.current_position, p))
            sorted_data = [(p, s) for p, s in zip(pigeon_data, sorted_pigeon)]
            sorted_pigeon_data = [p for p, _ in sorted(sorted_data, key=lambda x: self.hw_ctl.distance(self.hw_ctl.current_position, x[1]))]
            #print(sorted_pigeon_data[0], sorted_pigeon[0])
            for s in sorted_pigeon:
                pigeon_data = sorted_pigeon_data.pop(0)
                self.hw_ctl.position_adjust(s, detected_objects, pigeon_data)
            #print(pigeon_pos.left)
            #self.hw_ctl.moveServo((960, 540))
                self.hw_ctl.move_circle(pigeon_data)
        else:
            #print("No pigeon")
            self.hw_ctl.laser_off()
            #self.hw_ctl.moveServo(self.hw_ctl.init_position)
    

    def video_capture(self, raw_frame_queue, preprocessed_frame_queue, darknet_height, darknet_width):
        cap = cv2.VideoCapture("/dev/video0")
        #cap = cv2.VideoCapture(self.input_path)
        #cap = cv2.VideoCapture("/dev/video0")

        while cap.isOpened() and not self.stop_flag.is_set():
			
            ret, frame = cap.read()
            if not ret:
                break
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            f_h,f_w , _ = frame.shape
            print(f'frame h : {f_h} ,frame h : {f_w} ' )
            frame_resized = cv2.resize(frame_rgb, (darknet_width, darknet_height), interpolation=cv2.INTER_LINEAR)
            raw_frame_queue.put(frame)
            img_for_detect = darknet.make_image(darknet_width, darknet_height, 3)
            darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
            preprocessed_frame_queue.put(img_for_detect)
        self.stop_flag.set()
        cap.release()

    def inference(self, preprocessed_frame_queue, detections_queue, network, class_names):
    	while not self.stop_flag.is_set():
            darknet_image = preprocessed_frame_queue.get()
            prev_time = time.time()
            detections = darknet.detect_image(network, class_names, darknet_image, thresh=self.thresh)
            detections_queue.put(detections)
            darknet.free_image(darknet_image)

    @staticmethod
    def convert2relative(bbox, preproc_h, preproc_w):
        """
        YOLO format use relative coordinates for annotation
        """
        x, y, w, h = bbox
        print('this is ', x , y, w, h)
        print('preproc', x / preproc_w, y / preproc_h, w / preproc_w, h / preproc_h)
        return x / preproc_w, y / preproc_h, w / preproc_w, h / preproc_h
        #return x, y, w, h

    @staticmethod
    def convert2original(image, bbox, preproc_h, preproc_w):
        x, y, w, h = DarknetObjectDetector.convert2relative(bbox, preproc_h, preproc_w)


        image_h, image_w, __ = image.shape
        image_h = image_h
        image_w = image_w
        image_h = 480
        image_w = 640
        #print(f"image_h : {image_h}  image_w : {image_w}")

        orig_x = int(x * image_w)
        #print(f"x : {x}  orig_x : {orig_x}")
        orig_y = int(y * image_h)
        #print(f"y : {y}  orig_y : {orig_y}")
        orig_width = int(w * image_w)
        #print(f"w : {w}  orig_width : {orig_width}")
        orig_height = int(h * image_h)

        #bbox_converted = (orig_x, orig_y, orig_width, orig_height)
        bbox_converted = (orig_x, orig_y, orig_width, orig_height)
        print('this orig', image_h, image_w)
        return bbox_converted	

    def drawing(self, raw_frame_queue, preprocessed_frame_queue, detections_queue, fps_queue,
                darknet_height, darknet_width, class_colors):
        while not self.stop_flag.is_set():
            frame = raw_frame_queue.get()
            detections = detections_queue.get()

            detections_adjusted = []
            if frame is not None and detections:
                for label, confidence, bbox in detections:
                    # ปรับขนาดตำแหน่งของ bbox ให้เหมาะสมกับภาพเดิม
                    bbox_adjusted = self.convert2original(frame, bbox, darknet_height, darknet_width)
                    # เพิ่ม MapObject เข้าใน DetectedObjectList
                    obj = MapObject(label, float(confidence), *bbox_adjusted)
                    self.detected_objects.add_object(obj)
                    
                    #print(f'*bbox_adjusted from obj : {obj}')
                    
                    # เพิ่มข้อมูลตรวจจับที่ปรับแล้วเข้าไปใน detections_adjusted
                    detections_adjusted.append((str(label), confidence, bbox_adjusted))
            else:
                #print('Not Found')
                pass

            # วาดกรอบและข้อความบนภาพ
            image = darknet.draw_boxes(detections_adjusted, frame, class_colors)
            print(f'detections_adjusted : {detections_adjusted}')
            detections_adjusted = []
            self.to_another(self.detected_objects)
            self.detected_objects.clear()
            # แสดงภาพผ่านทางหน้าต่าง
            cv2.namedWindow("Inference",cv2.WINDOW_NORMAL)
            #cv2.resizeWindow("Inference",960,540)
            cv2.imshow("Inference", image)
            # หากมีการกดปุ่ม ESC ให้ออกจาก loop
            if cv2.waitKey(1) == 27:
                break
        cv2.destroyAllWindows()



    def run(self):
        self.restart_nvargus_daemon()
        darknet_width = 416     #512
        darknet_height = 416    #512
        cap = cv2.VideoCapture("/dev/video0")
        #cap.set(cv2_CAP_PROP_FRAME_WIDTH,960)
        #cap.set(cv2_CAP_PROP_FRAME_HEIGHT,540)
        #v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=H264
        


#--------when using web cam--------------------------------------------
        #video_width = 960
        #video_height = 540
#--------------------------------------end-web-cam------------------------
#--------when using csi cam------------------------------------------

        #video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#--------------------------------------csi-cam------------------------
        video_fps = int(cap.get(cv2.CAP_PROP_FPS))
        cap.release()
        del cap

        raw_frame_queue = queue.Queue()
        preprocessed_frame_queue = queue.Queue(maxsize=1)
        detections_queue = queue.Queue(maxsize=1)
        fps_queue = queue.Queue(maxsize=1)

        network, class_names, class_colors = darknet.load_network(
            self.config_file,
            self.data_file,
            self.weights,
            batch_size=1)

        exec_units = (
            threading.Thread(target=self.video_capture,
                             args=(raw_frame_queue, preprocessed_frame_queue, darknet_height, darknet_width)),
            threading.Thread(target=self.inference,
                             args=(preprocessed_frame_queue, detections_queue, network, class_names)),
            threading.Thread(target=self.drawing,
                             args=(raw_frame_queue, preprocessed_frame_queue, detections_queue, fps_queue,
                                   darknet_height, darknet_width, class_colors)),
        )
        for exec_unit in exec_units:
            exec_unit.start()
        for exec_unit in exec_units:
            exec_unit.join()

        print("\nDone.")
