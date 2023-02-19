from vision.ssd.mobilenetv1_ssd import create_mobilenetv1_ssd, create_mobilenetv1_ssd_predictor
import cv2
import numpy as np
import os

model_path ='models/voc/mb1-ssd-Epoch-49-Loss-1.6554838791996442.pth'
label_path = 'models/voc/labels.txt'
path = 'data/voc/JPEGImages/'
dir_list = os.listdir(path)
i = 1
image_path = path+dir_list[i]
class_names = [name.strip() for name in open(label_path).readlines()]
net = create_mobilenetv1_ssd(len(class_names), is_test=True)
net.load(model_path)
predictor = create_mobilenetv1_ssd_predictor(net, candidate_size=200)
orig_image = cv2.imread(image_path)
image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2RGB)
boxes, labels, probs = predictor.predict(image, 10, 0.4)
b = boxes.numpy()
max = np.max(b, axis=0)
print("max", max)
min = np.min(b, axis=0)
print("min",min)
for i in range(boxes.size(0)):
    box = boxes[i, :].numpy().astype(int)
    cv2.rectangle(orig_image, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 4)
    label = f"{class_names[labels[i]]}: {probs[i]:.2f}"
    cv2.putText(orig_image, label,
                (box[0] + 20, box[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,  # font scale
                (255, 0, 255),
                2)  # line type
path = "run_ssd_example_output.jpg"
cv2.imwrite(path, orig_image)
print(f"Found {len(probs)} objects. The output image is {path}")
