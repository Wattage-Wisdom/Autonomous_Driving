Files for processing image and detecting what traffic light is present.
Currently there are two methods, one using color masks to identify how many
pixels of a specific color are present and the other using machine leanring
to generate traffic light predictions. For machine learning, the CNN_1 
directory stores a trained CNN that achieved 93.5% accuracy based on 
test images acquired from the raspberry pi camera on the turtlebot. All
files were developed in Google Colab as .ipynb but also downloaded as a .py
and included in the directory. .py files will need to be adjusted to remove 
former integration with Google Drive. Can include .py file in package or just 
take functions in this file and integrate them inside proper node.
