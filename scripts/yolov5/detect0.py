# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/LNwODJXcvt4'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""

import argparse
import os
from pathlib import Path

from geometry_msgs.msg import PoseStamped

from utils.augmentations import (Albumentations, augment_hsv, classify_albumentations, classify_transforms, copy_paste,
                                 letterbox, mixup, random_perspective)
import numpy as np
import torch
import rospy
from sensor_msgs.msg import Image
import sys
sys.path.append('/home/ylc/cv_bridge_ws/install/lib/python3/dist-packages')

from cv_bridge import CvBridge

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
from models.common import DetectMultiBackend
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.torch_utils import select_device, smart_inference_mode

fx = 606.31005859375
fy = 606.111572265625
cx = 324.8408508300781
cy = 248.92527770996094

@smart_inference_mode()
def run(
        weights=ROOT / 'yolov5s.pt',  # model path or triton URL
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
):

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    bs = 1  # batch_size

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())

    bridge = CvBridge()
    target_pub = rospy.Publisher('/target_position', PoseStamped, queue_size=10)

    while True:
        image = rospy.wait_for_message('/camera/color/image_raw', Image)
        image_depth = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)
        image_depth = CvBridge().imgmsg_to_cv2(image_depth, desired_encoding="passthrough")
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        im0s = cv_image  # 读取图像
        im = letterbox(im0s, 640, stride=32, auto=True)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous

        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            pred = model(im, augment=augment, visualize=visualize)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        for i, det in enumerate(pred):  # per image
            seen += 1
            im0 = im0s.copy()

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Write results
                max_confidence_bbox = None  # 用于存储具有最大置信度的边界框
                max_confidence = 0.0  # 用于存储最大置信度的值

                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = names[c] if hide_conf else f'{names[c]}'
                    confidence = float(conf)
                    confidence_str = f'{confidence:.2f}'

                    if confidence > max_confidence:
                        max_confidence = confidence
                        max_confidence_bbox = xyxy

                # 如果存在具有最大置信度的边界框，则进行处理
                if max_confidence_bbox is not None:
                    x1, y1, x2, y2 = max_confidence_bbox
                    x = int((x1 + x2) / 2)
                    y = int((y1 + y2) / 2)
                    print(x, y, max_confidence)

                    # 显示图像
                    color = (0, 255, 0)  # BGR格式的颜色，这里是绿色
                    thickness = 2  # 边界框线宽
                    im0 = cv2.rectangle(im0, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                    im0 = cv2.putText(im0, f'{label} {confidence_str}', (int(x1), int(y1) - 5),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    depth_value = image_depth[y][x]

                    camera_x = ((x - cx) * depth_value / fx)/1000
                    camera_y = ((y - cy) * depth_value / fy)/1000
                    camera_z = (depth_value)/1000
                    print(camera_x,camera_y,camera_z)
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "lebai_base_link"
                    target_pose.pose.position.x = camera_x
                    target_pose.pose.position.y = camera_y
                    target_pose.pose.position.z = camera_z
                    target_pose.pose.orientation.x = 0.0
                    target_pose.pose.orientation.y = 0.0
                    target_pose.pose.orientation.z = 0.0
                    target_pose.pose.orientation.w = 1.0

                    target_pub.publish(target_pose)
                    cv2.imshow('Detection Result', im0)
                    cv2.waitKey(1)
            else:
                print("No bounding box found with confidence greater than zero.")
                cv2.imshow('Detection Result', im0)
                cv2.waitKey(1)


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'runs/train/exp4/weights/best.pt', help='model path or triton URL')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt


def main(opt):
    rospy.init_node('plasticdection', anonymous=True)
    run(**vars(opt))


if __name__ == '__main__':
    opt = parse_opt()
    main(opt)
