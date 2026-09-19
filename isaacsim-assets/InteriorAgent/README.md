---
viewer: false
license: other
license_name: interioragent-terms-of-use
license_link: >-
  https://kloudsim-usa-cos.kujiale.com/InteriorAgent/InteriorAgent_Terms_of_Use.pdf
---
# InteriorAgent: Interactive USD Interior Scenes for Isaac Sim-based Simulation

**InteriorAgent** is a collection of high-quality 3D USD assets specifically designed for indoor simulation in NVIDIA Isaac Sim environments. Each asset is structured with modular materials, scene description files, and physics-ready geometry, enabling fast integration for embodied AI and robotics tasks such as navigation, manipulation, and layout understanding.

<div align="center">
  <img src="https://kloudsim-usa-cos.kujiale.com/InteriorAgent/texture1.png" alt="InteriorAgent scene" width="80%"/>
  <p>A sample scene from the InteriorAgent dataset rendered in Isaac Sim. The scene features high-quality 3D assets such as sofas, cushions, tables, and chandeliers, all modeled with real-world scale. The bottom panel shows loaded asset files (e.g., <code>kuijiale_0021.usda</code>), and the right panel displays a hierarchical list of all 3D objects along with their <em>semantic labels</em>, supporting spatial reasoning and interaction in embodied AI tasks.</p>
</div>

## 🚀 Features

- ✅ Fully compatible with **Isaac Sim 4.2** and **4.5** on both **Windows** and **Linux**.
- 🎮 Built for real-time simulation, supports **interactive physical agents**.
- 🧱 Material system based on **NVIDIA MDL** (Material Definition Language), ensures photorealistic rendering and cross-version compatibility.
- 📦 Provided in `.usd` and `.usda` format with structured folders for **materials**, **meshes**, **lighting**, and **floorplan**.

---

## 🗂 Directory Structure

The dataset is organized per scene. Each scene folder follows the structure below:

```
kujiale_xxxx/
├── .thumbs/ # Optional thumbnail or cache folder (can be ignored)
├── Materials/ # Material library
│ ├── Textures/ # Texture images (optional, omitted here)
│ ├── *.mdl # MDL material and instance files
├── Meshes/ # Mesh geometry (e.g., .usd or .obj)
├── kujiale_xxxx.usda # Top-level USD scene file
├── limpopo_golf_course_4k.hdr # Environment lighting HDR file
└── rooms.json # Room-level metadata and spatial layout (JSON format)
```

### 🧭 Room Metadata (rooms.json)
Each scene folder includes a rooms.json file that defines the 2D floorplan layout of the space. It contains a list of room entries, where each room is defined by:

room_type: the semantic label (e.g., "living_room", "bedroom", "balcony", etc.)

polygon: a list of 2D coordinates representing the room's floor boundary in world coordinates

### 📌 Example
```
{
    "room_type": "balcony",
    "polygon": [
        [-0.3784970703125, -6.55287060546875],
        [4.005734375,      -6.55287060546875],
        [4.005734375,      -4.8603486328125],
        [-0.3784970703125, -4.8603486328125]
    ]
}
```

This represents a balcony room with a rectangular floorplan defined by a clockwise polygon in the Isaac Sim world coordinate system (X-Y plane). The polygon can be visualized or parsed using any geometric library (e.g., Shapely) to determine area, intersection, adjacency, etc.

### 🧪 Integration Tips
The coordinate system is consistent with Isaac Sim’s world frame: X is forward, Y is right, Z is upward.

Room geometry can be directly loaded using libraries like `Shapely` for spatial reasoning or map generation.

📦 Usage in Python
```
from shapely.geometry import Polygon
import json

with open("rooms.json", "r") as f:
    rooms = json.load(f)

for room in rooms:
    poly = Polygon(room["polygon"])
    print(f"Room: {room['room_type']}, Area: {poly.area}")
```

<div align="center">
  <img src="https://kloudsim-usa-cos.kujiale.com/InteriorAgent/texture2.png" alt="InteriorAgent structure overview" width="80%"/>
  <p>A hierarchical view of structural elements in an InteriorAgent scene. All architectural components are grouped under four main semantic categories: <code>ceiling</code>, <code>wall</code>, <code>floor</code>, and <code>other</code> (including <code>door</code> and <code>window</code>).</p>
</div>

## 🛠 Compatibility

- ✅ Tested with:  
  - Isaac Sim v4.2
  - Isaac Sim v4.5
  - Operating Systems: Windows 10/11, Ubuntu 22.04
- 🔧 MDL materials tested with Omniverse RTX renderer.
- 🌐 All files are offline usable and require no additional dependencies.

## 🏠 Citation

If you use InteriorAgent in your research or development, please cite or link to our project page:

```
@misc{InteriorAgent2025,
  title        = {InteriorAgent: Interactive USD Interior Scenes for Isaac Sim-based Simulation},
  author       = {SpatialVerse Research Team, Manycore Tech Inc.},
  year         = {2025},
  howpublished = {\url{https://huggingface.co/datasets/spatialverse/InteriorAgent}}
}
```

## 📄 License
This dataset is released under [InteriorAgent](https://kloudsim-usa-cos.kujiale.com/InteriorAgent/InteriorAgent_Terms_of_Use.pdf) License.
