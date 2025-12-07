# Dynamic Object Removal from LiDAR Point Clouds

Real-time dynamic object removal benchmark for LiDAR-based mapping and localization.

## Overview

This project benchmarks state-of-the-art **non-learning based** methods for dynamic object detection and removal from LiDAR point clouds.

## Benchmark Methods

| Method | Year | Venue | Approach | GitHub |
|--------|------|-------|----------|--------|
| **Dynablox** | 2023 | RA-L | Volumetric (Voxblox-based), High-confidence free-space | [ethz-asl/dynablox](https://github.com/ethz-asl/dynablox) |
| **DUFOMap** | 2024 | RA-L | Volumetric (UFOMap-based), Efficient dynamic awareness | [KTH-RPL/dufomap](https://github.com/KTH-RPL/dufomap) |

### Method Details

#### Dynablox
- **Paper**: "Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments"
- **Key Idea**: Incrementally estimate high-confidence free-space by modeling sensor noise, sparsity, and state drift
- **Performance**: 86% IoU at 17 FPS on DOALS dataset

#### DUFOMap
- **Paper**: "DUFOMap: Efficient Dynamic Awareness Mapping"
- **Key Idea**: Efficient UFOMap-based dynamic object detection
- **Performance**: Improved efficiency over Dynablox

## Dataset

### DOALS (Dynamic Object Aware LiDAR SLAM) Dataset

- **Source**: ETH Zurich ASL
- **Download**: http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes
- **Paper**: Pfreundschuh et al., "Dynamic Object Aware LiDAR SLAM based on Automatic Generation of Training Data", ICRA 2021

#### Sequences

| Location | Sequences | Environment | Description |
|----------|-----------|-------------|-------------|
| Hauptgebaeude | 2 | Indoor | ETH main building |
| Station | 2 | Multi-level | Zurich main station |
| Shopville | 2 | Indoor | Shopping area |
| Niederdorf | 2 | Outdoor | Pedestrian zone |
| Simulated | 1 | Synthetic | Cars, people, animals, various objects |

#### Sensor Specifications
- **LiDAR**: Ouster OS1-64 (Gen 1)
- **Frequency**: 10 Hz
- **Points per revolution**: 2048
- **Recording**: Handheld (ego-motion distortion present)

#### Ground Truth
- 10 manually annotated frames per sequence
- Dynamic point indices stored in `indices.csv`
- Format: `timestamp_ns, idx1, idx2, idx3, ...`

## Evaluation Metrics

### Primary Metrics

| Metric | Formula | Description |
|--------|---------|-------------|
| **IoU** | $\frac{TP}{TP + FP + FN}$ | Intersection over Union |
| **Precision** | $\frac{TP}{TP + FP}$ | Dynamic detection accuracy |
| **Recall** | $\frac{TP}{TP + FN}$ | Dynamic detection coverage |

### Additional Metrics

| Metric | Description |
|--------|-------------|
| **FPS** | Frames per second (real-time capability) |
| **Preservation Rate** | Static points correctly preserved |
| **Rejection Rate** | Dynamic points correctly removed |

## Project Structure

```
dynamic_cloud_removal_lidar/
├── README.md
├── data/
│   └── DOALS/
│       ├── hauptgebaeude/
│       │   ├── sequence_1/
│       │   │   ├── bag.bag
│       │   │   └── indices.csv
│       │   └── sequence_2/
│       ├── station/
│       ├── shopville/
│       └── niederdorf/
├── src/
│   ├── methods/
│   │   ├── dynablox/
│   │   └── dufomap/
│   ├── utils/
│   │   ├── rosbag_parser.py
│   │   ├── pointcloud_utils.py
│   │   └── evaluation.py
│   └── benchmark.py
├── configs/
│   ├── dynablox.yaml
│   └── dufomap.yaml
├── results/
│   └── ...
└── scripts/
    ├── download_doals.sh
    └── run_benchmark.sh
```

## Installation

```bash
# Clone repository
git clone https://github.com/93won/dynamic_cloud_removal_lidar.git
cd dynamic_cloud_removal_lidar

# Install dependencies
pip install -r requirements.txt
```

### Dependencies

```
numpy
open3d
rosbags  # ROS-free rosbag parsing
pyyaml
matplotlib
tqdm
```

## Usage

### 1. Download Dataset

```bash
./scripts/download_doals.sh /path/to/data/DOALS
```

### 2. Run Benchmark

```bash
python src/benchmark.py --config configs/benchmark.yaml
```

### 3. Evaluate Results

```bash
python src/utils/evaluation.py --results results/
```

## TODO

- [ ] Implement rosbag parser (ROS-free)
- [ ] Implement Dynablox (C++ / Python wrapper)
- [ ] Implement DUFOMap (C++ / Python wrapper)
- [ ] Evaluation pipeline
- [ ] Visualization tools
- [ ] Add more methods (ERASOR, Removert)

## References

### Dynablox
```bibtex
@article{schmid2023dynablox,
  title={Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments},
  author={Schmid, Lukas and Andersson, Olov and Sulser, Aurelio and Pfreundschuh, Patrick and Siegwart, Roland},
  journal={IEEE Robotics and Automation Letters},
  volume={8},
  number={10},
  pages={6259--6266},
  year={2023}
}
```

### DUFOMap
```bibtex
@article{duberg2024dufomap,
  title={DUFOMap: Efficient Dynamic Awareness Mapping},
  author={Duberg, Daniel and Zhang, Qingwen and Jia, Ming and Jensfelt, Patric},
  journal={IEEE Robotics and Automation Letters},
  year={2024}
}
```

### DOALS Dataset
```bibtex
@inproceedings{pfreundschuh2021doals,
  title={Dynamic Object Aware LiDAR SLAM based on Automatic Generation of Training Data},
  author={Pfreundschuh, Patrick and Hendrikx, Hubertus FC and Reijgwart, Victor and Dub{\'e}, Renaud and Siegwart, Roland and Cramariuc, Andrei},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  pages={11641--11647},
  year={2021}
}
```

## License

BSD-3-Clause

## Author

- **93won** - [GitHub](https://github.com/93won)
