# whi_qrcode_generator
QR code generator supports QR and ArUco

## Dependency
OpenCV >= 4.8.0, refer to the [official guidance](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) for installation

## Param
```
<!-- common -->
<param name="type" default="qr"/> <!-- type of code: qr or aruco -->
<param name="image_size" type="int" default="500"/>
<param name="output_path" type="str" default="$(env HOME)/Desktop/"/>
<param name="contents" type="str" default="hello world"/>
<param name="show_generated" type="bool" default="true"/>
<!-- aruco -->
<param name="marker_size" default="4"/>
<!-- qr -->
<param name="code_size" type="int" default="200"/>
<param name="correction_level" type="str" default="middle"/>
```

## Usage
### QR
```
roslaunch whi_qrcode_generator whi_qrcode_generator.launch image_size:=500 contents:="this is a demo"
```

After running this command, an encoded image(png format) named the content "this is a demo" with size of 500x500 will be saved in the given path "/${HOME}/Desktop"

![this is a demo](https://github.com/xinjuezou-whi/whi_qrcode_generator/assets/72239958/90c2865e-4c6f-4cd8-a6bc-8cd6fae1866c)

For content with quotes, like JSON, enclose the content with apostrophes. For example, say we'd like to encode the JSON data:

{"translate": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0]}:
```
roslaunch whi_qrcode_generator whi_qrcode_generator.launch image_size:=500 output_path:="$HOME/Desktop" contents:='{"translate": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0]}'
```

![json_code](https://github.com/xinjuezou-whi/whi_qrcode_generator/assets/72239958/c3f232e6-e838-42f2-95be-6b82a58bbda4)

### ArUco
```
roslaunch whi_qrcode_generator whi_qrcode_generator.launch type:=aruco marker_size:=4 contents:=0
```

After running this command, an encoded image(png format) named the content "0" with size of 500x500 will be saved in the given path "/${HOME}/Desktop"

![0](https://github.com/user-attachments/assets/5f270ccf-7263-4e4a-94b9-ed02274ffc56)

Currently only the predefined dictionary is supported:
| marker size | max marker number | dictionary    |
|-------------|-------------------|---------------|
| 4           | 50                | DICT_4X4_50   |
| 4           | 100               | DICT_4X4_100  |
| 4           | 250               | DICT_4X4_250  |
| 4           | 1000              | DICT_4X4_1000 |
| 5           | 50                | DICT_5X5_50   |
| 5           | 100               | DICT_5X5_100  |
| 5           | 250               | DICT_5X5_250  |
| 5           | 1000              | DICT_5X5_1000 |
| 6           | 50                | DICT_6X6_50   |
| 6           | 100               | DICT_6X6_100  |
| 6           | 250               | DICT_6X6_250  |
| 6           | 1000              | DICT_6X6_1000 |
| 7           | 50                | DICT_7X7_50   |
| 7           | 100               | DICT_7X7_100  |
| 7           | 250               | DICT_7X7_250  |
| 7           | 1000              | DICT_7X7_1000 |
