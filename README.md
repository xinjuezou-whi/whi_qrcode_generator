# whi_qrcode_generator
QR code generator

## Dependency
OpenCV >= 4.8.0, refer to the [official guidance](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) for installation

## Param
```
image_size default=500 # code image size in pixel
correction_level default="middle" #low, middle, quality, high
output_path default="/home/whi/Desktop/" # the path of the output image
contents default="hello world" # contents of code
show_generated default="true" # wheter to show generated code
```

## Usage
```
roslaunch whi_qrcode_generator whi_qrcode_generator.launch image_size:=500 output_path:="/home/Desktop" contents:="this is a demo" 
```

After running this command, an encoded image(png format) named the content "this is a demo" with size of 500x500 will be saved in the given path "/home/Desktop"

![this is a demo](https://github.com/xinjuezou-whi/whi_qrcode_generator/assets/72239958/90c2865e-4c6f-4cd8-a6bc-8cd6fae1866c)

