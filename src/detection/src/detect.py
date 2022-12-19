#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

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

def CLIPprocess(image, text):
    with torch.no_grad():
        logits_per_image, logits_per_text = model(image, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()
        id = np.argmax(probs)
    return probs, id

# def showReslut(img_arr, 
#             text,
#             fps = None,
#             resize = (480, 640), 
#             org = (20,90),
#             fontFace = cv2.FONT_HERSHEY_SIMPLEX,
#             fontScale = 1,
#             color = (255,255,255),
#             thickness = 1,
#             lineType = cv2.LINE_AA):

#     img_arr = cv2.resize(img_arr, resize, interpolation=cv2.INTER_AREA)
#     cv2.putText(img_arr, text, org, fontFace, fontScale, color, thickness, lineType)
#     if fps is not None:
#         fpsText = 'FPS: {:.1f}'.format(fps)
#         cv2.putText(img_arr, fpsText, (300, 600), fontFace, fontScale, color, thickness, lineType)

#     cv2.imshow('Image', img_arr)

def buildOptions(target):
    targetFloor = target.split()[0][0]
    targetList = [targetFloor + str(num).rjust(2, "0") for num in range(1, 40)]
    return targetList

##################################################################
## zed2 input

def detecting(cv_image):
    print("Hahahahahahaha")
    img_pil = Image.fromarray(cv_image)
    image = preprocess(img_pil).unsqueeze(0).to(device)

    # Region Proposal
    boxes, rpnScores = RPN(img_pil, rpnModel)

    if len(boxes) == None:
        print("No proper region to pay attention")
    else: 
        for box in boxes:
            x1, y1, x2, y2 =  box
            img_crop = img_pil.crop((int(x1), int(y1), int(x2), int(y2)))
            image = preprocess(img_crop).unsqueeze(0).to(device)
            sceneProbs, sceneId = CLIPprocess(image, tokenScene)
            conf = np.max(sceneProbs)
            if  conf > classThres:
                text = '{}({:.1f}%)'.format(sceneList[sceneId], conf*100)
                print(text)

                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (20, 20, 225), 3)
                cv2.putText(cv_image, text, 
                            (int(x1), int(y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (200, 52, 52), 
                            2, lineType=cv2.LINE_AA)
                if sceneList[sceneId] == 'doorplate':
                    targrtProbs, targetId = CLIPprocess(image, tokenTarget)
                    if targetList[targetId] == target:
                        print("Reach the target room")
                    else:
                        print("Keep searching")

                print("=====================================")

    cv_image = cv2.resize(cv_image, (640, 360), interpolation=cv2.INTER_AREA)
    cv2.imshow("vision", cv_image) 
    cv2.waitKey(1)

def image_callback(img):
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(img)
    detecting(cv_image)

if __name__ == '__main__':
    ## config 
    selectModel = "ViT-L/14@336px" # ViT-L/14@336px, ViT-L/14", ViT-B/16 
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, preprocess = clip.load(selectModel, device=device) 
    classThres = 0.85
    filePath = rospkg.RosPack().get_path(name = 'detection')
    with open(filePath + '/src/detect.json') as f:
        cfg = json.load(f)

    sceneDict = cfg['sceneList']
    sceneList = list(sceneDict.values())
    # tokenScene= clip.tokenize(sceneList).to(device)
    tokenScene = torch.cat([clip.tokenize(f"a photo of the {c}") for c in sceneList]).to(device)

    target = '412'
    targetList = buildOptions(target)
    tokenTarget = torch.cat([clip.tokenize(f"numbers with {c}") for c in targetList]).to(device)

    # choose: fasterrcnn_mobilenet_v3_large_fpn, fasterrcnn_mobilenet_v3_large_320_fpn
    rpnModel = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn()    
     
    rospy.init_node('detect_node', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color/compressed", CompressedImage, image_callback, queue_size = 1)
    rospy.spin()

