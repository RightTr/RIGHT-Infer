
# 简介

本仓库基于shouxieai的[infer](https://github.com/shouxieai/infer)，实现物体的识别和分割，并提取点云信息。

* 支持Azure Kinect DK [Azure-Kinect-Sensor-SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)  

    支持Realsense D435i [librealsense](https://github.com/IntelRealSense/librealsense)

* 支持YOLOv8、YOLOv8-Seg
  
* C++接口，多线程

# 环境配置

本项目依赖于OpenCV、TensorRT、PCL、CUDA库。参考CMakeLists.txt安装相关依赖和指定路径配置。

# 使用方法

<details>
<summary>YOLOv8</summary>

详细参考[YOLOv8推理详解及部署实现](https://blog.csdn.net/qq_40672115/article/details/134276907)

1、下载YOLOv8

```bash
git clone https://github.com/ultralytics/ultralytics.git
```

2、修改YOLOv8源码  

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

3、导出onnx

```python
from ultralytics import YOLO
import onnx
model = YOLO("/home/right/Infer/workspace/best.pt")
success = model.export(format="onnx", dynamic=True, simplify=True)
```

4、进入TensoRT路径，生成engine文件

* 进入TensorRT-8.6.1.6/targets/x86_64-linux-gnu/bin

* 使用trtexec将onnx文件进行转换  

    记得取消换行

```bash
./trtexec --onnx=best.onnx --saveEngine=best.engine
```

</details>

<details>
<summary>YOLOv8-Seg</summary>  

详细参考[YOLOv8-Seg推理详解及部署实现](https://blog.csdn.net/qq_40672115/article/details/134277752)

1、下载YOLOv8  

```bash
git clone https://github.com/ultralytics/ultralytics.git
```

2、修改YOLOv8源码  

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
        return (torch.cat([x, mc], 1).permute(0, 2, 1), p) if self.export else (torch.cat([x[0], mc], 1), (x[1], mc, p)) #这里和推文不一样
```

3、导出onnx

```python
from ultralytics import YOLO
import onnx
model = YOLO("/home/right/Infer/workspace/best_seg.pt")
success = model.export(format="onnx", dynamic=True, simplify=True)
```

4、进入TensoRT路径，生成engine文件

* 进入TensorRT-8.6.1.6/targets/x86_64-linux-gnu/bin

* 使用trtexec将onnx文件进行转换  

    记得取消换行

```bash
./trtexec --onnx=best_seg.onnx --saveEngine=best_seg.transd.engine
```
