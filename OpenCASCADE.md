# OpenCASCAD学习笔记
太折磨了

## OpenCASCADE命名方式
OpenCASCADE的命名**前缀**是有规律的。最基础的如下

`gp`：是“Geometric Primitives”，即图元，这里面定义的是最基础的图形学元素，例如点，向量和矩阵。

`Geom`：就是“Geometry”的缩写，这里面定义的是较为高级的几何元素，例如点，曲线，面。

`TopoDS`：即“Topological Data Structure”，用这个开头的包才是BRep模型。上面的都只是几何表达，没有模型“实体”。

`BRepBuilderAPI`：用来创建TopoDS的接口

## OpenCASCADE模型创建流程

一般而言`TopoDS`，比如`TopoDS_Edge`，`TopoDS_Wire`，才是我们要的模型，一个模型的创建过程如下：

1. 定义基础元素例如`gp_Pnt`，`pn_Lin`等

2. 构成几何形状如`Geom_Curve`，`Geom_TrimmedCurve`

3. 构造BRepBuilder`BRepBuilderAPI`

4. 利用构造器`BRepBuilderAPI`构造`TopoDS`

## Geom类
所有的`Geom_Geometry`类都继承自`Standard_Transient`

`Geom_Geometry`有3个主要的子类，分别是`Geom_Point`,`Geom_Curve`,`Geom_Surface`

具体属于哪一个类的判断函数有两个

+ `DynamicType()`用来获取当前变量类名称
+ `isKind`用来获取父类名称

## MakeBottle例子
