import argparse
import os
import random
import time
import shutil
import warnings
import datetime
import torch
import torchvision
import torch.nn as nn
import torch.nn.parallel
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
import torch.optim
import torch.utils.data
import torchvision.datasets as datasets
import torchvision.transforms as transforms
import torchvision.models as models
from reshape import reshape_model
import cv2
import matplotlib.pyplot as plt
import numpy as np

def imshow(img):
    img = img / 2 + 0.5     # unnormalize
    npimg = img.numpy()
    plt.imshow(np.transpose(npimg, (1, 2, 0)))
    plt.savefig('my_plot.png', dpi=300)

data_path = 'data/cat_dog/test/'
arch = 'resnet18'
resolution = 224
model = models.__dict__[arch]()
model = reshape_model(model, arch, 2)
model_path = torch.load("models/cat_dog/model_best.pth.tar")
model.load_state_dict(model_path['state_dict'])
# setup data transformations
normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                    std=[0.229, 0.224, 0.225])
train_transforms = transforms.Compose([
    transforms.RandomResizedCrop(resolution),
    transforms.RandomHorizontalFlip(),
    transforms.ToTensor(),
    normalize,
])

train_dataset = datasets.ImageFolder(data_path, train_transforms)
train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=1, shuffle=True,
        num_workers=1, pin_memory=True)
dataiter = iter(train_loader)
images, labels = next(dataiter)
# device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
# images = images.to(device)
# model.to(device)
outputs = model(images)
_, predicted = torch.max(outputs, 1)
print("outputs", outputs)
print("predicted", predicted)
# imshow(torchvision.utils.make_grid(images))