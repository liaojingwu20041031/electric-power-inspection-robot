# 02 — CAD Geometry Fact Map

```
AUDIT_CLASS   READ_ONLY
CAD_MODIFIED  NO
SOURCE        CAD/Retail-Cart-3D-Model/
```

## 1. Inventory

```
66  .SLDPRT     SolidWorks part          stp/
22  .3MF        3D-printing package      stl/
 3  .SLDASM     SolidWorks assembly      sldasm/
 1  readme.txt
```

The three assemblies are the interesting ones: `小车底盘.SLDASM` (chassis, 1 716 731 B),
`万向轮.SLDASM` (caster), `机械爪.SLDASM` (gripper).

## 2. Format accessibility — the decisive finding

| format | count | parseable here | why |
|---|---|---|---|
| 3MF | 22 | **YES** | ZIP + XML, `unit="millimeter"` declared in the model |
| SLDPRT | 66 | **NO** | SolidWorks proprietary container |
| SLDASM | 3 | **NO** | same |

SLDPRT was probed rather than assumed:

```
magic bytes        43991d2f00000004
OLE2 compound doc  False
strings dimension hints  0 matches
```

It is not an OLE2 compound file, so generic OLE tooling cannot open it either, and
there is no plain-text dimension metadata to scrape. Available Python CAD readers
were checked and none is installed: `trimesh`, `cadquery`, `OCC`, `steputils`,
`ezdxf` all absent, and installing software is out of scope for this round.

```
CAD_GEOMETRY_AVAILABLE  PARTIAL  (3MF only)
```

Stated precisely, because the distinction changes what the owner must do:

> The drive-wheel and chassis-assembly geometry **exists in the repository but is
> unreadable in this environment**. That is not the same as the data being absent.

## 3. The 22 parseable 3MF meshes, with real bounding boxes

All values read from the mesh vertices, unit millimetre as declared in the file.

| part | X | Y | Z |
|---|---|---|---|
| 前半圆 / 前半圆X | 178.00 | 235.00 | 153.00 |
| 后半圆 / 后半圆X | 178.00 | 233.00 | 233.00 |
| 盖板 | 196.00 | 255.00 | 21.00 |
| 双目垫板 (stereo camera shim) | 249.95 | 30.00 | 5.00 |
| 激光雷达固定架 (LiDAR bracket) | 120.00 | 120.00 | 22.00 |
| 英伟达固定板X (Jetson plate) | 127.00 | 94.00 | 8.50 |
| 伸缩杆固定底座 | 53.00 | 220.00 | 150.00 |
| 后半圆连接-固定件 | 240.00 | 99.91 | 6.00 |
| 连接-固定件 | 6.00 | 70.00 | 240.00 |
| 前后半圆连接件 | 185.00 | 40.00 | 3.00 |
| 后半圆固定架 | 170.00 | 19.00 | 32.50 |
| 底盘连接处支架X | 42.00 | 100.00 | 140.00 |
| 支架垫片 | 120.00 | 40.00 | 20.00 |
| 模块固定 | 107.50 | 45.00 | 45.00 |
| 电池固定 | 100.00 | 65.00 | 31.00 |
| 电池门 | 57.34 | 15.55 | 56.00 |
| 稳压板底座 | 114.50 | 54.00 | 100.00 |
| 语音播报模块固定盖 | 133.49 | 136.98 | 3.00 |
| 语音播报模块外挂支架 | 133.49 | 78.00 | 136.98 |
| 伸缩杆支架盖子 | 3.00 | 85.00 | 50.00 |

## 4. What the 3MF set can and cannot supply

Every one of the 22 is a **3D-printed bracket, cover, shell or shim**. Not one is a
drive wheel, an axle, a motor, or a load-bearing chassis member.

Can supply: bracket envelope dimensions. `激光雷达固定架` at 120×120×22 mm bounds
the LiDAR mounting plate, and `双目垫板` at 249.95×30×5 mm bounds the stereo shim.

Cannot supply: **sensor extrinsics**. A bracket's bounding box states how big the
bracket is, not where it sits on the vehicle or how the sensor is oriented in it.
Extrinsics need the assembly, which is format-locked.

## 5. Parts that hold the answers, and are locked

| part | what it would settle | status |
|---|---|---|
| `stp/电机轮.SLDPRT` (motor wheel, 156 150 B) | drive wheel radius, tread width | format-locked |
| `stp/电机.SLDPRT` (motor) | axle position within the drive unit | format-locked |
| `sldasm/小车底盘.SLDASM` (chassis assembly) | **wheel track** — the left/right drive wheel centres | format-locked |
| `sldasm/万向轮.SLDASM` (caster) | caster geometry for the simulation | format-locked |
| `stp/驱动器.SLDPRT` (driver unit) | mounting envelope | format-locked |

So the two parameters this whole audit chain is about — wheel radius and wheel
track — are each answerable from CAD in principle and unreadable in practice here.

## 6. Mass, COM and inertia are not obtainable from any of this

```
CAD_GEOMETRY_AVAILABLE != CAD_PHYSICAL_DYNAMICS_AVAILABLE
```

Even the readable 3MF meshes carry geometry only — no material, no density, no mass
property. Inferring mass or inertia by assuming a density for a printed shell would
be fabrication, and it is explicitly not done here. The SolidWorks parts may hold
material assignments, but they are unreadable, so that cannot be claimed either.

## 7. Recommended unlock path, cheaper than measuring

Opening `小车底盘.SLDASM` and `电机轮.SLDPRT` on any machine with SolidWorks and
exporting STEP or STL would very likely settle wheel radius, tread width and wheel
track without touching the vehicle, and would also give the sensor bracket
positions needed for extrinsics.

This should be attempted **before** the physical-measurement route, because it is
lower effort, non-intrusive, and would leave a far smaller residual measurement
list. It is recorded as the first recommendation in doc 05.
