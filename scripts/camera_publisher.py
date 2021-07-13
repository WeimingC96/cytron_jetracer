#!/usr/bin/env python3

from jetcam.csi_camera import CSICamera

camera = CSICamera(width=224, height=224)
#We can then capture a frame from the camera like this

image = camera.read()

print(image.type)
print(type(image))