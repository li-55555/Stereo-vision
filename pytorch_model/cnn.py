import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import matplotlib as plt
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from multiprocessing import freeze_support
from torch.optim import SGD
from torch.optim.lr_scheduler import StepLR

datatrain_dir="data/train"
dataval_dir="data/val"

class CNN(nn.Module):
    def __init__(self,classes):
        super(CNN, self).__init__()
        self.conv1=nn.Conv2d(in_channels=3, out_channels=16, kernel_size=3, stride=2, padding=1)
        self.conv2=nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.conv3=nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, stride=2, padding=1)
        self.fc1=nn.Linear(64*6*6,100)
        self.fc2=nn.Linear(100,classes)
    
    def forward(self,x):
        x=F.relu(self.conv1(x))
        x=F.relu(self.conv2(x))
        x=F.relu(self.conv3(x))
        x=x.view(-1,64*6*6)
        x=F.relu(self.fc1(x))
        x=self.fc2(x)
        return x

def train(net,optim,criterion,train_dataloader):
    running_loss = 0.0
    for data in train_dataloader:
        input,target=data
        output=net(input)
        loss=criterion(output,target)
        
        optim.zero_grad()
        loss.backward()
        optim.step()
        running_loss += loss.item()
    return running_loss / len(train_dataloader)

def test(net,loader,criterion):
    net.eval()
    running_loss = 0.0
    correct = 0
    total = 0
    with torch.no_grad():
        for data in loader:
            inputs, labels = data
            inputs, labels = inputs, labels
            outputs = net(inputs)
            loss = criterion(outputs, labels)
            running_loss += loss.item()
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
    return running_loss / len(loader), 100 * correct / total

if __name__ == '__main__':
    freeze_support()
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    train_transformer=transforms.Compose([transforms.Resize(60),
                                    transforms.RandomCrop(48),
                                    transforms.ToTensor(),
                                    transforms.Normalize((0.5,0.5,0.5),(0.5,0.5,0.5))])

    val_transformer=transforms.Compose([transforms.Resize(48),
                                    transforms.ToTensor(),
                                    transforms.Normalize((0.5,0.5,0.5),(0.5,0.5,0.5))])

    train_dataset=datasets.ImageFolder(datatrain_dir,train_transformer)
    val_dataset=datasets.ImageFolder(dataval_dir,val_transformer)

    train_dataloader=torch.utils.data.DataLoader(train_dataset,batch_size=64,shuffle=False,num_workers=1)
    val_dataloader=torch.utils.data.DataLoader(val_dataset,batch_size=64,shuffle=False,num_workers=1)
    
    net=CNN(2)
    
    optim = SGD(net.parameters(),0.01,0.9)
    criterion=torch.nn.CrossEntropyLoss()
    lr_step=StepLR(optim,step_size=50,gamma=0.1)
    epochs=200
    
    for epoch in range(0,epochs):
        train_loss=train(net,optim,criterion,train_dataloader)
        test_loss,test_acc=test(net,train_dataloader,criterion)
        print("train_loss: " +str(train_loss)+" test_loss: "+str(test_loss)+"test_acc"+str(test_acc))
        
    torch.save(net,'model.pth')
    