#!/usr/bin/env python
# -*- coding: utf-8 -*
"""
 Copyright (C) 2018-2019 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
from __future__ import print_function, division

import logging
import os
import sys
from argparse import ArgumentParser, SUPPRESS
from math import exp as exp
from time import time

import cv2
from openvino.inference_engine import IECore
import numpy as np

import pyrealsense2 as rs
import socket
import struct

# wzy global variable start >>>

cameraMatrix = np.zeros([3,3],dtype=np.float)
distCoeffs = np.zeros((5), dtype=np.float)
outputRvecRaw = np.zeros((3), dtype=float)
outputTvecRaw = np.zeros((3), dtype=float)

# D455 color 640x480 30hz parameter
fx = 380.831
fy = 379.965
cx = 309.269
cy = 238.532

# wzy global variable end <<<

logging.basicConfig(format="[ %(levelname)s ] %(message)s", level=logging.INFO, stream=sys.stdout)
log = logging.getLogger()
# -------------------------------- tcp configure ----------------------------- #
server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
server.bind(('127.0.0.1',8000))
server.listen(5)
print("waiting msg ...")
conn, clint_add = server.accept()

def build_argparser():
    parser = ArgumentParser(add_help=False)
    args = parser.add_argument_group('Options')
    args.add_argument('-h', '--help', action='help', default=SUPPRESS, help='Show this help message and exit.')
    args.add_argument("-m", "--model", help="Required. Path to an .xml file with a trained model.",
                      required=True, default='frozen_darknet_yolov3_model.xml', type=str)
    args.add_argument("-i", "--input", help="Required. Path to an image/video file. (Specify 'cam' to work with "
                                            "camera)", required=True, default='cam', type=str)
    args.add_argument("-l", "--cpu_extension",
                      help="Optional. Required for CPU custom layers. Absolute path to a shared library with "
                           "the kernels implementations.", type=str, default=None)
    args.add_argument("-d", "--device",
                      help="Optional. Specify the target device to infer on; CPU, GPU, FPGA, HDDL or MYRIAD is"
                           " acceptable. The sample will look for a suitable plugin for device specified. "
                           "Default value is CPU", default="HDDL", type=str)
    args.add_argument("--labels", help="Optional. Labels mapping file", default=None, type=str)
    args.add_argument("-t", "--prob_threshold", help="Optional. Probability threshold for detections filtering",
                      default=0.5, type=float)
    args.add_argument("-iout", "--iou_threshold", help="Optional. Intersection over union threshold for overlapping "
                                                       "detections filtering", default=0.4, type=float)
    args.add_argument("-ni", "--number_iter", help="Optional. Number of inference iterations", default=1, type=int)
    args.add_argument("-pc", "--perf_counts", help="Optional. Report performance counters", default=False,
                      action="store_true")
    args.add_argument("-r", "--raw_output_message", help="Optional. Output inference results raw values showing",
                      default=False, action="store_true")
    args.add_argument("--no_show", help="Optional. Don't show output", action='store_true')
    return parser


class YoloParams:
    # ------------------------------------------- Extracting layer parameters ------------------------------------------
    # Magic numbers are copied from yolo samples
    def __init__(self, param, side):
        self.num = 3 if 'num' not in param else int(param['num'])
        self.coords = 4 if 'coords' not in param else int(param['coords'])
        self.classes = 1 if 'classes' not in param else int(param['classes'])
        self.side = side
        self.anchors = [10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0,
                        198.0,
                        373.0, 326.0] if 'anchors' not in param else [float(a) for a in param['anchors'].split(',')]

        self.isYoloV3 = False

        if param.get('mask'):
            mask = [int(idx) for idx in param['mask'].split(',')]
            self.num = len(mask)

            maskedAnchors = []
            for idx in mask:
                maskedAnchors += [self.anchors[idx * 2], self.anchors[idx * 2 + 1]]
            self.anchors = maskedAnchors

            self.isYoloV3 = True # Weak way to determine but the only one.

    def log_params(self):
        k = 1
        #params_to_print = {'classes': self.classes, 'num': self.num, 'coords': self.coords, 'anchors': self.anchors}
        #[log.info("         {:8}: {}".format(param_name, param)) for param_name, param in params_to_print.items()]


def entry_index(side, coord, classes, location, entry):
    side_power_2 = side ** 2
    n = location // side_power_2
    loc = location % side_power_2
    return int(side_power_2 * (n * (coord + classes + 1) + entry) + loc)


def scale_bbox(x, y, h, w, class_id, confidence, h_scale, w_scale):
    xmin = int((x - w / 2) * w_scale)
    ymin = int((y - h / 2) * h_scale)
    xmax = int(xmin + w * w_scale)
    ymax = int(ymin + h * h_scale)
    return dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, class_id=class_id, confidence=confidence)


def parse_yolo_region(blob, resized_image_shape, original_im_shape, params, threshold):
    # ------------------------------------------ Validating output parameters ------------------------------------------
    _, _, out_blob_h, out_blob_w = blob.shape
    assert out_blob_w == out_blob_h, "Invalid size of output blob. It sould be in NCHW layout and height should " \
                                     "be equal to width. Current height = {}, current width = {}" \
                                     "".format(out_blob_h, out_blob_w)

    # ------------------------------------------ Extracting layer parameters -------------------------------------------
    orig_im_h, orig_im_w = original_im_shape
    resized_image_h, resized_image_w = resized_image_shape
    objects = list()
    predictions = blob.flatten()
    side_square = params.side * params.side

    # ------------------------------------------- Parsing YOLO Region output -------------------------------------------
    for i in range(side_square):
        row = i // params.side
        col = i % params.side
        for n in range(params.num):
            obj_index = entry_index(params.side, params.coords, params.classes, n * side_square + i, params.coords)
            scale = predictions[obj_index]
            if scale < threshold:
                continue
            box_index = entry_index(params.side, params.coords, params.classes, n * side_square + i, 0)
            # Network produces location predictions in absolute coordinates of feature maps.
            # Scale it to relative coordinates.
            x = (col + predictions[box_index + 0 * side_square]) / params.side
            y = (row + predictions[box_index + 1 * side_square]) / params.side
            # Value for exp is very big number in some cases so following construction is using here
            try:
                w_exp = exp(predictions[box_index + 2 * side_square])
                h_exp = exp(predictions[box_index + 3 * side_square])
            except OverflowError:
                continue
            # Depends on topology we need to normalize sizes by feature maps (up to YOLOv3) or by input shape (YOLOv3)
            w = w_exp * params.anchors[2 * n] / (resized_image_w if params.isYoloV3 else params.side)
            h = h_exp * params.anchors[2 * n + 1] / (resized_image_h if params.isYoloV3 else params.side)
            for j in range(params.classes):
                class_index = entry_index(params.side, params.coords, params.classes, n * side_square + i,
                                          params.coords + 1 + j)
                confidence = scale * predictions[class_index]
                if confidence < threshold:
                    continue
                objects.append(scale_bbox(x=x, y=y, h=h, w=w, class_id=j, confidence=confidence,
                                          h_scale=orig_im_h, w_scale=orig_im_w))
    return objects


def intersection_over_union(box_1, box_2):
    width_of_overlap_area = min(box_1['xmax'], box_2['xmax']) - max(box_1['xmin'], box_2['xmin'])
    height_of_overlap_area = min(box_1['ymax'], box_2['ymax']) - max(box_1['ymin'], box_2['ymin'])
    if width_of_overlap_area < 0 or height_of_overlap_area < 0:
        area_of_overlap = 0
    else:
        area_of_overlap = width_of_overlap_area * height_of_overlap_area
    box_1_area = (box_1['ymax'] - box_1['ymin']) * (box_1['xmax'] - box_1['xmin'])
    box_2_area = (box_2['ymax'] - box_2['ymin']) * (box_2['xmax'] - box_2['xmin'])
    area_of_union = box_1_area + box_2_area - area_of_overlap
    if area_of_union == 0:
        return 0
    return area_of_overlap / area_of_union


def main():
    global fx, fy, cx, cy, plane_x, plane_y, plane_z
    # args = build_argparser().parse_args() # wzy change
    parser = ArgumentParser(add_help=False)
    args = parser.add_argument_group('Options')
    args.cpu_extension = False
    args.model = 'frozen_darknet_yolov3_model.xml'
    args.bin = 'frozen_darknet_yolov3_model.bin'
    args.device = 'MYRIAD'
    args.labels = 'frozen_darknet_yolov3_model.mapping'
    args.input = 'cam'
    args.prob_threshold = 0.8
    args.iou_threshold = 0.6
    args.raw_output_message = True
    args.no_show = False

    # ------------- 1. Plugin initialization for specified device and load extensions library if specified -------------
    log.info("Creating Inference Engine...")
    ie = IECore()
    if args.cpu_extension and 'CPU' in args.device:
        ie.add_extension(args.cpu_extension, "CPU")

    # -------------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) --------------------
    log.info("Loading network")
    # wzy change
    #net = ie.read_network(args.model, os.path.splitext(args.model)[0] + ".bin")
    # net = ie.read_network('frozen_darknet_yolov3_model.xml', "frozen_darknet_yolov3_model.bin")
    net = ie.read_network(args.model,args.bin)

    # ---------------------------------- 3. Load CPU extension for support specific layer ------------------------------
    if "CPU" in args.device:
        supported_layers = ie.query_network(net, "CPU")
        not_supported_layers = [l for l in net.layers.keys() if l not in supported_layers]
        if len(not_supported_layers) != 0:
            log.error("Following layers are not supported by the plugin for specified device {}:\n {}".
                      format(args.device, ', '.join(not_supported_layers)))
            log.error("Please try to specify cpu extensions library path in sample's command line parameters using -l "
                      "or --cpu_extension command line argument")
            sys.exit(1)

    assert len(net.inputs.keys()) == 1, "Sample supports only YOLO V3 based single input topologies"

    # ---------------------------------------------- 4. Preparing inputs -----------------------------------------------

    log.info("Preparing inputs")
    input_blob = next(iter(net.inputs))

    #  Defaulf batch_size is 1
    net.batch_size = 1

    # Read and pre-process input images
    n, c, h, w = net.inputs[input_blob].shape

    if args.labels:
        with open(args.labels, 'r') as f:
            labels_map = [x.strip() for x in f]
    else:
        labels_map = None
    # with open('frozen_darknet_yolov3_model.mapping', 'r') as f:
    #     labels_map = [x.strip() for x in f]

    # input_stream = 0 if args.input == "cam" else args.input
    #
    # is_async_mode = True
    # cap = cv2.VideoCapture(input_stream)
    #
    # number_input_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # number_input_frames = 1 if number_input_frames != -1 and number_input_frames < 0 else number_input_frames
    #
    # wait_key_code = 1
    #
    # # Number of frames in picture is 1 and this will be read in cycle. Sync mode is default value for this case
    # if number_input_frames != 1:
    #     ret, frame = cap.read()
    # else:
    #     is_async_mode = False
    #     wait_key_code = 0

    # ----------------------------------------- 5. Loading model to the plugin -----------------------------------------
    log.info("Loading model to the plugin")
    exec_net = ie.load_network(network=net, num_requests=2, device_name=args.device)

    cur_request_id = 0
    next_request_id = 1
    render_time = 0
    parsing_time = 0

    # ----------------------------------------------5.1 realsense input video stream -----------------------------------
    # Configure depth and color streams
    pipeline = rs.pipeline()
    # 创建 config 对象：
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    get_img_flag = False
    count = 0
    while not (get_img_flag and count > 10): # 多取几幅图片，前几张不清晰
        # Wait for a coherent pair of frames（一对连贯的帧）: depth and color
        frames = pipeline.wait_for_frames()
        print('wait for frames in the first loop')
        get_color_frame = frames.get_color_frame()

        if not get_color_frame: # 如果color没有得到图像，就continue继续
            continue

        color_frame = np.asanyarray(get_color_frame.get_data())
        get_img_flag = True # 跳出循环
        count += 1
    # ----------------------------------------------- 6. Doing inference -----------------------------------------------
    log.info("Starting inference...")
    print("To close the application, press 'CTRL+C' here or switch to the output window and press ESC key")
    print("To switch between sync/async modes, press TAB key in the output window")
    is_async_mode = True
    try:
        while True:
            # Here is the first asynchronous point: in the Async mode, we capture frame to populate the NEXT infer request
            # in the regular mode, we capture frame to the CURRENT infer request

            # >>>>>>>>>>>  calculate time sum >>>>>>>>>>>>>>>#
            cpu_start = time()
        #    print('--------------------------new loop---------------------------------')
            # if is_async_mode:
            #     ret, next_frame = cap.read()
            # else:
            #     ret, frame = cap.read()
            #
            # if not ret:
            #     break
            # Wait for a coherent pair of frames（一对连贯的帧）: depth and color
            next_frames = pipeline.wait_for_frames()

            get_next_color_frame = next_frames.get_color_frame()
            next_color_frame = np.asanyarray(get_next_color_frame.get_data())
            # cv2.imshow("color", next_color_frame)


            if is_async_mode:
                request_id = next_request_id
                in_frame = cv2.resize(color_frame, (w, h))
            else:
                request_id = cur_request_id
                in_frame = cv2.resize(color_frame, (w, h))

            # resize input_frame to network size
            in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
            in_frame = in_frame.reshape((n, c, h, w))

            # Start inference
            start_time = time()
            exec_net.start_async(request_id=request_id, inputs={input_blob: in_frame})
            det_time = time() - start_time

            # Collecting object detection results
            objects = list()
            if exec_net.requests[cur_request_id].wait(-1) == 0:
                output = exec_net.requests[cur_request_id].outputs
                start_time = time()
                for layer_name, out_blob in output.items():
                    out_blob = out_blob.reshape(net.layers[net.layers[layer_name].parents[0]].shape)
                    layer_params = YoloParams(net.layers[layer_name].params, out_blob.shape[2])
                   # log.info("Layer {} parameters: ".format(layer_name))
                    layer_params.log_params()
                    objects += parse_yolo_region(out_blob, in_frame.shape[2:],
                                                 next_color_frame.shape[:-1], layer_params,
                                                 args.prob_threshold)
                parsing_time = time() - start_time

            # Filtering overlapping boxes with respect to the --iou_threshold CLI parameter
            objects = sorted(objects, key=lambda obj : obj['confidence'], reverse=True)
            for i in range(len(objects)):
                if objects[i]['confidence'] == 0:
                    continue
                for j in range(i + 1, len(objects)):
                    if intersection_over_union(objects[i], objects[j]) > args.iou_threshold:
                        objects[j]['confidence'] = 0

            # Drawing objects with respect to the --prob_threshold CLI parameter
            objects = [obj for obj in objects if obj['confidence'] >= args.prob_threshold]

            #if len(objects) and args.raw_output_message:
               # log.info("\nDetected boxes for batch {}:".format(1))
               # log.info(" Class ID | Confidence | XMIN | YMIN | XMAX | YMAX | COLOR ")

            origin_im_size = color_frame.shape[:-1]

            count = 1

            for obj in objects:
                #print('for obj count:')
                #print(count)
                count = count + 1
                # Validation bbox of detected object
                if obj['xmax'] > origin_im_size[1] or obj['ymax'] > origin_im_size[0] or obj['xmin'] < 0 or obj['ymin'] < 0:
                    continue
                # color = (int(min(obj['class_id'] * 12.5, 255)),
                #          min(obj['class_id'] * 7, 255), min(obj['class_id'] * 5, 255))
                color = (100, 100, 150)
                det_label = labels_map[obj['class_id']] if labels_map and len(labels_map) >= obj['class_id'] else \
                    str(obj['class_id'])

               # if args.raw_output_message:
                  #  log.info(
                  #      "{:^9} | {:10f} | {:4} | {:4} | {:4} | {:4} | {} ".format(det_label, obj['confidence'], obj['xmin'],
                     #                                                             obj['ymin'], obj['xmax'], obj['ymax'],
                       #                                                           color))

                cv2.rectangle(color_frame, (obj['xmin'], obj['ymin']), (obj['xmax'], obj['ymax']), color, 2)
                cv2.putText(color_frame,
                    "#" + str(obj['class_id']) + ' ' + str(round(obj['confidence'] * 100, 1)) + ' %',
                    (obj['xmin'], obj['ymin'] - 7), cv2.FONT_HERSHEY_COMPLEX, 0.6, color, 1)


               # print("obj['class_id']: ", obj['class_id'])

                send_data_byte = bytes(0)
                if obj['class_id'] == 0:  # 0 is target, 1 is pickup
                    target_leftup_rightdown_corner = [0, obj['xmin'], obj['ymin'], obj['xmax'], obj['ymax']]
                    for i in range(len(target_leftup_rightdown_corner)):
                        # print(target_leftup_rightdown_corner[i])
                        target_senddata = str(target_leftup_rightdown_corner[i]) + ','
                        # print(target_senddata.encode())
                        send_data_byte += target_senddata.encode()
                        # print(send_data_byte)
                    conn.send(send_data_byte)

                if obj['class_id'] == 1:
                    pickup_leftup_rightdown_corner = [1, obj['xmin'], obj['ymin'], obj['xmax'], obj['ymax']]
                    for i in range(len(pickup_leftup_rightdown_corner)):
                        # print(pickup_leftup_rightdown_corner[i])
                        pickup_senddata = str(pickup_leftup_rightdown_corner[i]) + ','
                        # print(pickup_senddata.encode())
                        send_data_byte += pickup_senddata.encode()
                        # print(send_data_byte)
                    conn.send(send_data_byte)


            send_data_byte = bytes(0)
            if len(objects) == 0:
                pickup_leftup_rightdown_corner = [-1, 0, 0, 0, 0]
                for i in range(len(pickup_leftup_rightdown_corner)):
                    #print(pickup_leftup_rightdown_corner[i])
                    pickup_senddata = str(pickup_leftup_rightdown_corner[i]) + ','
                    # print(pickup_senddata.encode())
                    send_data_byte += pickup_senddata.encode()
                    # print(send_data_byte)
                conn.send(send_data_byte)

            # Draw performance stats over frame
            inf_time_message = "Inference time: N\A for async mode" if is_async_mode else \
                "Inference time: {:.3f} ms".format(det_time * 1e3)
            render_time_message = "OpenCV rendering time: {:.3f} ms".format(render_time * 1e3)
            async_mode_message = "Async mode is on. Processing request {}".format(cur_request_id) if is_async_mode else \
                "Async mode is off. Processing request {}".format(cur_request_id)
            parsing_message = "YOLO parsing time is {:.3f} ms".format(parsing_time * 1e3)

            cv2.putText(color_frame, inf_time_message, (15, 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (200, 10, 10), 1)
            cv2.putText(color_frame, render_time_message, (15, 45), cv2.FONT_HERSHEY_COMPLEX, 0.5, (10, 10, 200), 1)
            cv2.putText(color_frame, async_mode_message, (10, int(origin_im_size[0] - 20)), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                        (10, 10, 200), 1)
            cv2.putText(color_frame, parsing_message, (15, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (10, 10, 200), 1)

            start_time = time()
            if not args.no_show:
                cv2.imshow("DetectionResults", color_frame)

            render_time = time() - start_time

            if is_async_mode:
                cur_request_id, next_request_id = next_request_id, cur_request_id
                color_frame = next_color_frame

            if not args.no_show:
                #key = cv2.waitKey(wait_key_code)
                key = cv2.waitKey(1)

                # ESC key
                if key == 27:
                    break
                # Tab key
                if key == 9:
                    exec_net.requests[cur_request_id].wait()
                    is_async_mode = not is_async_mode
                  #  log.info("Switched to {} mode".format("async" if is_async_mode else "sync"))

            cpu_end = time()
            #print('>>>>>>>>>>>>>>>>>>>>>>>>>>cpu time :  ', cpu_end-cpu_start)
    finally:
        pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main() or 0)
