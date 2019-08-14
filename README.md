## Requirements
```
Python 3.6.8(3.6.X)
  - Numpy == 1.16.4
  - Scipy == 1.3.0
  - Matplotlib == 3.1.1(可选)
```

## Introduction

```
./
    base.py 
        包含振动信号类和测点基类
    mixin.py 
        包含各个故障模式的Mixin
    simulators.py 
        包含各个故障模式信号的仿真生成

./Blender or ./Compressor
    equipment.py 
        设备静态信息类，只提供部分属性，供测点类引用
    measure_points.py 
        各个设备测点的实现类
    .test/ 
        文件夹包含符合接口的诊断函数以及测试用例
```

## Usage

```bash
pip install -r requirements.txt

#根目录下
python ./compressor/test/motor_driven_ver.py
```


# Test Sample
```
Chlorine.csv
    Col1 motor_drive_x	
    Col2 motor_drive_y	
    Col3 motor_nondrive_x
    Col4 motor_nondrive_y
    Col5 gearbox_in_bearing_x
    Col6 gearbox_in_bearing_y
    Col7 gearbox_iss
    Col8 gearbox_out_bearing_x
    Col9 gearbox_out_bearing_y
    Col10 Chlorine_compressor_drive_x
    Col11 Chlorine_compressor_drive_y
    Col12 Chlorine_compressor_nondrive_x
    Col13 Chlorine_compressor_nondrive_y
    Col14 Chlorine_compressor_nondrive_z

Prepolymerizer.csv
    Col1 motor_drive_x
    Col2 motor_drive_y
    Col3 motor_nondrive_x
    Col4 down_gearbox_in
    Col5 down_gearbox_out
    Col6 seal_bearing
```
