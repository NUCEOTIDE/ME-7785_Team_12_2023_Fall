import torch
import numpy as np
import cv2
import cv2
import os
from crop_img import crop_img
import torch
import torch.nn as nn
from sklearn.metrics import accuracy_score, confusion_matrix
import argparse

class nn_model(nn.Module):
    def __init__(self):
        super(nn_model,self).__init__()
        self.act = nn.LeakyReLU()
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=10, kernel_size=(5,5), stride=(2,2))
        self.maxpool = nn.MaxPool2d(kernel_size=2)
        self.conv2 = nn.Conv2d(in_channels=10, out_channels=20, kernel_size=(5,5),stride=(2,2))
        self.fc = nn.Linear(in_features=500, out_features=5)

    def forward(self, data):
        batch_size = data.size()[0]
        x = self.conv1(data)
        x = self.act(x)
        x = self.maxpool(x)
        x = self.conv2(x)
        x = self.act(x)
        x = self.maxpool(x)
        x = x.view(batch_size,-1)
        x = self.fc(x)
        return x

# change root to the name of dataset directory
parser = argparse.ArgumentParser()
parser.add_argument('-d','--dataset',type=str)
parser.add_argument('-m','--model',type=str)
args = parser.parse_args()
root = '2023Fimgs' if args.dataset is None else args.dataset
model_name = 'cnn_all.pth' if args.model is None else args.model + '.pth'
data = []
pred = []
device = 'cpu'
model = torch.load(model_name).to(device)

# pick the image files from all the files under root and sort them
file_list = os.listdir(root)
image_list = []
for file in file_list:
    if not file[-3:] == 'txt':
        image_list.append(file)
image_list.sort(key=lambda x : int(x[:-4]))

# iterate
for file in image_list:
    path = os.path.join(root,file)
    data.append(path)
    # read original image
    img = cv2.imread(path)
    # crop image
    cropped_image = crop_img(img)
    # class 0
    if cropped_image is None:
        pred.append(0)
    # class is not 0, need further analysis
    else:
        # convert opencv image type to torch tensor
        img = cv2.resize(cropped_image,(100,100))
        numpy_image = np.array(cv2.split(img))
        torch_image = torch.from_numpy(numpy_image)
        input = torch.unsqueeze(torch_image,0).to(torch.float32)

        # get model output
        output = model.forward(input)
        label = torch.argmax(output).item() + 1
        pred.append(label)

# get targets from labels.txt and sort the targets
target = []
label_file = os.path.join(root,'labels.txt')
with open(label_file) as f:
    txt = f.readlines()
for line in txt:
    im, label = line.split(', ')
    target.append((int(im),int(label)))
target.sort(key=lambda x:x[0])
target = [i[1] for i in target]

# output accuracy score and confusion matrix
acc = accuracy_score(target,pred)
cm = confusion_matrix(target,pred)
print('accuracy score in ' + root + ' dataset:',acc)
print('confusion matrix:')
print(cm)

