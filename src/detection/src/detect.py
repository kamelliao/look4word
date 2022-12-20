#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String, Float64MultiArray, Int8
from cv_bridge import CvBridge

import torch
import clip
from PIL import Image
import json
import numpy as np
import cv2
import torchvision 
from torchvision import transforms
from torchvision.ops import nms

def RPN(img, model, detectThres = 0.15, iouThres = 0.3):
    convert_tensor = transforms.ToTensor()
    img_tensor = convert_tensor(img)

    model.eval()
    preds = rpnModel([img_tensor])[0]
    boxes = preds['boxes']
    scores = preds['scores']
    boxes = boxes[scores >= detectThres]
    scores = scores[scores >= detectThres]
    final_boxesId = nms(boxes = boxes, scores = scores, iou_threshold = iouThres)
    final_boxes = boxes[final_boxesId]
    return final_boxes, scores

def CLIPprocess(img_crop, text):
    image = preprocess(img_crop).unsqueeze(0).to(device)

    with torch.no_grad():
        logits_per_image, logits_per_text = model(image, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()
        id = np.argmax(probs)
    return probs, id

def buildOptions(target):
    targetFloor = target[0]
    targetList = [targetFloor + str(num).rjust(2, "0") for num in range(1, 40)]
    return targetList

def getCenterAndPub(box):
    HFov = 110
    halfW = Rw / 2
    x1, y1, x2, y2 = box
    x = np.mean((x1, x2))
    y = np.mean((y1, y2))
    angle = -(round(x) - halfW) / halfW * HFov/2 
    array = Float64MultiArray(data = [x, y, angle])
    pubGoal.publish(array)
    print("Degree to turn:", angle)
    print('Object center point: ', [x, y])

def webcam_callback(web_img):
    cv_web_image = bridge.compressed_imgmsg_to_cv2(web_img)
    web_img_pil = Image.fromarray(cv_web_image)

    boxes, rpnScores = RPN(web_img_pil, rpnModel)
    if len(boxes) == None:
        print("No proper region to pay attention")
    else: 
        print("Number of boxes: ", len(boxes))
        for box in boxes.detach().numpy():
            x1, y1, x2, y2 =  box
            web_img_crop = web_img_pil.crop((int(x1), int(y1), int(x2), int(y2)))
            sceneProbs, sceneId = CLIPprocess(web_img_crop, tokenScene)
            conf = np.max(sceneProbs)
            if  conf > classThres:
                if sceneList[sceneId] == 'doorplate':
                    pubDetect.publish(4)
                    targetProbs, targetId = CLIPprocess(img_crop, tokenTarget)
                    
                    targetConf = np.max(targetProbs)
                    print(targetProbs[0][11])
                    if targetConf > classThres:
                        if targetList[targetId] == '412':
                            text = "Reach the target room"
                            print(text)
                            pubDetect.publish(5)
                        else:
                            print("Keep searching")
    cv2.putText(cv_web_image, text, 
                (int(x1), int(y1-10)),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 52, 52), 
                2, lineType=cv2.LINE_AA)
    cv2.imshow('webcam', cv_web_image)
    print("=====================================")

def detecting(cv_image):
    img_pil = Image.fromarray(cv_image)
    # img_pil = trans(img_pil)

    # Region Proposal
    boxes, rpnScores = RPN(img_pil, rpnModel)

    if len(boxes) == None:
        print("No proper region to pay attention")
    else: 
        print("Number of boxes: ", len(boxes))
        for box in boxes.detach().numpy():
            x1, y1, x2, y2 =  box
            img_crop = img_pil.crop((int(x1), int(y1), int(x2), int(y2)))
            sceneProbs, sceneId = CLIPprocess(img_crop, tokenScene)
            conf = np.max(sceneProbs)
            if  conf > classThres:
                text = '{}({:.1f}%)'.format(sceneList[sceneId], conf*100)
                rospy.loginfo(text)
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (20, 20, 225), 2)
                cv2.putText(cv_image, text, 
                            (int(x1), int(y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 52, 52), 
                            2, lineType=cv2.LINE_AA)

                if sceneList[sceneId] == 'elevator':
                    rospy.loginfo("Find an elevator")
                    pubDetect.publish(1)
                    getCenterAndPub(box)
                    print("Move to the elevator")
                if sceneList[sceneId] == 'elevator button':
                    rospy.loginfo("Find an elevator panel")
                    pubDetect.publish(2)
                    getCenterAndPub(box)
                    print("Move in front of the elevator button")
                if sceneList[sceneId] == 'door':
                    rospy.loginfo("Find a door")
                    pubDetect.publish(3)
                    getCenterAndPub(box)
                    print("Turn to face the door")   
                    rospy.Subscriber("webcam_img", CompressedImage, webcam_callback, queue_size = 1, buff_size = 65536*2048)
                    rospy.spin()

                print("=====================================")

    cv2.imshow("vision", cv_image) 
    cv2.waitKey(1)

def image_callback(img):
    # global resize_cv_imag
    cv_image = bridge.compressed_imgmsg_to_cv2(img)
    # resize_cv_imag = cv2.resize(cv_image, (Rw, Rh), interpolation=cv2.INTER_AREA)
    detecting(cv_image)

if __name__ == '__main__':
    Rh = 360
    Rw = 640
    resize_cv_imag = np.uint8(np.zeros((360, 640, 3)))
    trans = transforms.Resize((Rh, Rw))
    ## config 
    # ['RN50', 'RN101', 'RN50x4', 'RN50x16', 'RN50x64', 'ViT-B/32', 'ViT-B/16', 'ViT-L/14', 'ViT-L/14@336px']
    selectModel = "ViT-L/14"
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, preprocess = clip.load(selectModel, device=device) 
    classThres = 0.6
    filePath = rospkg.RosPack().get_path('detection')

    with open(filePath + '/src/detect.json') as f:
        cfg = json.load(f)

    sceneDict = cfg['sceneList']
    sceneList = list(sceneDict.values())
    tokenScene = torch.cat([clip.tokenize(f"a photo of the {c}") for c in sceneList]).to(device)
    
    target = '412'
    targetList = buildOptions(target)
    tokenTarget = torch.cat([clip.tokenize(f"numbers with {c}") for c in targetList]).to(device)

    # choose: fasterrcnn_resnet50_fpn, fasterrcnn_mobilenet_v3_large_fpn, fasterrcnn_mobilenet_v3_large_320_fpn
    rpnModel = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(weights='DEFAULT')    

    ## reording
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640,  480))
    ##################################################################################################################
    ## zed2 input
    bridge = CvBridge()
    pubDetect = rospy.Publisher("SeeSomething", Int8, queue_size = 1)
    pubGoal = rospy.Publisher("CenterAndDegree", Float64MultiArray, queue_size = 1)
    rospy.init_node('detect_node', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color/compressed", CompressedImage, image_callback, queue_size = 1, buff_size = 65536*4096)
    rospy.spin()

