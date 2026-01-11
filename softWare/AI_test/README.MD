## 🚀 Run Fall Detection AI (Conda)

```bash
# 1. Create & activate conda environment
conda create -n fall_ai python=3.9 -y
conda activate fall_ai

# 2. Install dependencies
pip install numpy pandas scikit-learn tensorflow==2.13.0

# 3. Train CNN1D model
cd src
python train.py

# 4. Export INT8 TFLite model (ESP32-ready)
python export_tflite.py

python export_kernels.py > kernels.h

gcc cnn_infer.c -O3 -o cnn.exe
