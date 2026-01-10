import numpy as np

# ================= CONFIG =================
WINDOW_SIZE = 50

# PHẢI GIỐNG THỨ TỰ LÚC TRAIN
FEATURES = ["pitch", "roll", "svm", "gz"]

# ================= SCALER =================
def load_scaler(path):
    """
    scaler.txt format:
    mean:
    v1,v2,v3,v4
    std:
    v1,v2,v3,v4
    """
    with open(path, "r") as f:
        lines = f.readlines()

    if len(lines) < 4:
        raise ValueError("❌ scaler.txt format invalid")

    mean = np.array(
        [float(x) for x in lines[1].strip().split(",")],
        dtype=np.float32
    )

    std = np.array(
        [float(x) for x in lines[3].strip().split(",")],
        dtype=np.float32
    )

    return mean, std


def normalize(data, mean, std):
    """
    data shape: (N, FEATURES)
    """
    return (data - mean) / (std + 1e-6)


# ================= WINDOW =================
def sliding_windows(data):
    """
    Generate sliding windows for realtime simulation
    data shape: (N, FEATURES)
    yield shape: (WINDOW_SIZE, FEATURES)
    """
    for i in range(len(data) - WINDOW_SIZE + 1):
        yield data[i:i + WINDOW_SIZE]


# ================= ESP32 HELPER =================
def preprocess_window(window, mean, std):
    """
    Single window normalize + reshape
    Output shape: (1, WINDOW_SIZE, FEATURES)
    """
    window = normalize(window, mean, std)
    return window.reshape(1, WINDOW_SIZE, len(FEATURES))
