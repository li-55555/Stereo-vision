import torch
import torchvision.transforms as transforms
from PIL import Image
from cnn import CNN

def load_image(filename): 

    transformer = transforms.Compose([
        transforms.Resize((48, 48)),
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    model = torch.load('model.pth')
    model.eval()

    classes = ['good_box', 'evil_box']

    img = Image.open(filename)
    img_tensor = transformer(img).unsqueeze(0)

    with torch.no_grad():
        output = model(img_tensor)
        _, predicted = torch.max(output.data, 1)
        print(classes[predicted.item()])
        
    return predicted.item()