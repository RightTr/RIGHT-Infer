
# Introduction

This repository is based on [infer](https://github.com/shouxieai/infer)，enabling object dectetion and segmentation, as well as stereo extraction.

* Supports Azure Kinect DK via [Azure-Kinect-Sensor-SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)  

    Supports Realsense D435i [librealsense](https://github.com/IntelRealSense/librealsense)

* Supports YOLOv8、YOLOv8-Seg
  
* Developed in C++

# Environment Setup

The project depends on libraries such as OpenCV, PCL, CUDA and TensorRT. Please refet to CMakeLists.txt to install the required dependencies.

# Usage

## Model Import

<details>
<summary>YOLOv8</summary>

For more details, please refer to[YOLOv8推理详解及部署实现](https://blog.csdn.net/qq_40672115/article/details/134276907)

1、Download ultralytics:

```bash
git clone https://github.com/ultralytics/ultralytics.git
```

2、Modify ultralytics source code:

* ultralytics/engine/exporter.py 458行
  
```python
        # output_names = ["output0", "output1"] if isinstance(self.model, SegmentationModel) else ["output0"]
        # dynamic = self.args.dynamic
        # if dynamic:
        #     dynamic = {"images": {0: "batch", 2: "height", 3: "width"}}  # shape(1,3,640,640)
        #     if isinstance(self.model, SegmentationModel):
        #         dynamic["output0"] = {0: "batch", 2: "anchors"}  # shape(1, 116, 8400)
        #         dynamic["output1"] = {0: "batch", 2: "mask_height", 3: "mask_width"}  # shape(1,32,160,160)
        #     elif isinstance(self.model, DetectionModel):
        #         dynamic["output0"] = {0: "batch", 2: "anchors"}  # shape(1, 84, 8400)

        output_names = ['output0', 'output1'] if isinstance(self.model, SegmentationModel) else ['output0']
        dynamic = self.args.dynamic
        if dynamic:
            dynamic = {'images': {0: 'batch'}}  # shape(1,3,640,640)
            if isinstance(self.model, SegmentationModel):
                dynamic['output0'] = {0: 'batch'}  # shape(1, 116, 8400)
                dynamic['output1'] = {0: 'batch'}  # shape(1,32,160,160)
            elif isinstance(self.model, DetectionModel):
                dynamic['output0'] = {0: 'batch', 2: 'anchors'}  # shape(1, 84, 8400)
```

* ultralytics/nn/modules/head.py
  
```python
    def forward(self, x):
        """Concatenates and returns predicted bounding boxes and class probabilities."""
        if self.end2end:
            return self.forward_end2end(x)

        for i in range(self.nl):
            x[i] = torch.cat((self.cv2[i](x[i]), self.cv3[i](x[i])), 1)
        if self.training:  # Training path
            return x
        y = self._inference(x)
        # return y if self.export else (y, x)
        return y.permute(0, 2, 1) if self.export else (y, x)
```

3、Export to ONNX:

```python
from ultralytics import YOLO
import onnx
model = YOLO("/home/right/Infer/workspace/best.pt")
success = model.export(format="onnx", dynamic=True, simplify=True)
```

4、Convert ONNX to TensorRT Engine:

* Navigate to TensorRT-8.6.1.6/targets/x86_64-linux-gnu/bin

* Run trtexec to convert ONNX model to TensorRT engine:  

```bash
./trtexec --onnx=best.onnx --saveEngine=best.engine
```

</details>

<details>
<summary>YOLOv8-Seg</summary>  

For more details, please refet to [YOLOv8-Seg推理详解及部署实现](https://blog.csdn.net/qq_40672115/article/details/134277752)

1、Download ultralytics:

```bash
git clone https://github.com/ultralytics/ultralytics.git
```

2、Modify ultralytics source code: 

* ultralytics/engine/exporter.py
  
```python
        # output_names = ["output0", "output1"] if isinstance(self.model, SegmentationModel) else ["output0"]
        # dynamic = self.args.dynamic
        # if dynamic:
        #     dynamic = {"images": {0: "batch", 2: "height", 3: "width"}}  # shape(1,3,640,640)
        #     if isinstance(self.model, SegmentationModel):
        #         dynamic["output0"] = {0: "batch", 2: "anchors"}  # shape(1, 116, 8400)
        #         dynamic["output1"] = {0: "batch", 2: "mask_height", 3: "mask_width"}  # shape(1,32,160,160)
        #     elif isinstance(self.model, DetectionModel):
        #         dynamic["output0"] = {0: "batch", 2: "anchors"}  # shape(1, 84, 8400)

        output_names = ['output0', 'output1'] if isinstance(self.model, SegmentationModel) else ['output0']
        dynamic = self.args.dynamic
        if dynamic:
            dynamic = {'images': {0: 'batch'}}  # shape(1,3,640,640)
            if isinstance(self.model, SegmentationModel):
                dynamic['output0'] = {0: 'batch'}  # shape(1, 116, 8400)
                dynamic['output1'] = {0: 'batch'}  # shape(1,32,160,160)
            elif isinstance(self.model, DetectionModel):
                dynamic['output0'] = {0: 'batch', 2: 'anchors'}  # shape(1, 84, 8400)
```

* ultralytics/nn/modules/head.py 

```python
    def forward(self, x):
        """Return model outputs and mask coefficients if training, otherwise return outputs and mask coefficients."""
        p = self.proto(x[0])  # mask protos
        bs = p.shape[0]  # batch size

        mc = torch.cat([self.cv4[i](x[i]).view(bs, self.nm, -1) for i in range(self.nl)], 2)  # mask coefficients
        x = Detect.forward(self, x)
        print(mc.shape, x.shape)
        if self.training:
            return x, mc, p
        x = x.transpose(1, 2)
        # return (torch.cat([x, mc], 1), p) if self.export else (torch.cat([x[0], mc], 1), (x[1], mc, p))
        return (torch.cat([x, mc], 1).permute(0, 2, 1), p) if self.export else (torch.cat([x[0], mc], 1), (x[1], mc, p)) # Note: This part is different from the blog post
```

3、Export to ONNX:

```python
from ultralytics import YOLO
import onnx
model = YOLO("/home/right/Infer/workspace/best_seg.pt")
success = model.export(format="onnx", dynamic=True, simplify=True)
```

4、Convert ONNX to TensorRT Engine:

* Navigate to TensorRT-8.6.1.6/targets/x86_64-linux-gnu/bin

* Run trtexec to convert ONNX model to TensorRT engine:

```bash
./trtexec --onnx=best_seg.onnx --saveEngine=best_seg.transd.engine
```

</details>

## Parameter Configuration
  
* kinect.yaml
    * Camera mounting position

    * Pointcloud processing parameters

* realsense.yaml

## 运行项目

* Run the Realsense process to start the 60Hz color stream coarse alignment. It acts as a TCP client to communicate with the Kinect process, controlling its thread operation modes.

```bash
./rs_process 
```

* Run the Kinect process to perform fine alignment of depth information. It serves as a TCP server, listens to the Realsense process, and uses the serial port to send the results of coarse or fine alignment.

```bash
./k4a_process
```

* run two processes in one command.
  
```bash
./run.sh
```