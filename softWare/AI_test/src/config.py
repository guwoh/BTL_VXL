# Sliding window
WINDOW_SIZE = 50
STEP_SIZE   = 25

# Features dùng cho CNN1D
FEATURES = ['svm', 'gx', 'gy', 'gz']

# Label mapping
LABEL_MAP = {
    "Normal": 0,
    "Fall": 1
}

NUM_CLASSES = 2

# Training
BATCH_SIZE = 32
EPOCHS = 30
