import numpy as np
import pandas as pd
import tensorflow as tf
from collections import Counter

MODEL_PATH = "../models/activity_cnn.h5"
SCALER_PATH = "../models/scaler.txt"
LABEL_PATH  = "../models/labels.txt"
CSV_PATH    = "../test_data/test.csv"

WINDOW = 50
STRIDE = 10
FEATURES = ["pitch", "roll", "svm", "gx", "gy", "gz"]

# ================= LOAD =================
def load_scaler(path):
    with open(path) as f:
        lines = f.readlines()
    mean = np.array(list(map(float, lines[1].split(","))))
    std  = np.array(list(map(float, lines[3].split(","))))
    return mean, std

def load_labels(path):
    labels = {}
    with open(path) as f:
        for line in f:
            i, name = line.strip().split(",")
            labels[int(i)] = name
    return labels

# ================= WINDOW =================
def sliding_window(data, win, stride):
    X = []
    for i in range(0, len(data) - win, stride):
        X.append(data[i:i+win])
    return np.array(X)

# ================= MAIN =================
def main():
    model = tf.keras.models.load_model(MODEL_PATH)
    mean, std = load_scaler(SCALER_PATH)
    labels = load_labels(LABEL_PATH)

    df = pd.read_csv(CSV_PATH)
    data = df[FEATURES].values.astype(np.float32)

    windows = sliding_window(data, WINDOW, STRIDE)

    preds = []
    confs = []

    for w in windows:
        x = (w - mean) / std
        x = np.expand_dims(x, axis=0)

        prob = model.predict(x, verbose=0)[0]
        cls  = np.argmax(prob)
        preds.append(labels[cls])
        confs.append(prob[cls])

    # ================= AGGREGATION =================
    total = len(preds)
    counter = Counter(preds)

    print("\n📊 WINDOW DISTRIBUTION")
    for k, v in counter.items():
        print(f"{k:10s}: {v} ({v/total:.1%})")

    # ===== FALL PRIORITY =====
    FALL_NAME = "Fall"
    fall_ratio = counter.get(FALL_NAME, 0) / total

    if fall_ratio >= 0.2:
        final_state = FALL_NAME
        reason = f"Fall ratio {fall_ratio:.1%} ≥ 20%"
    else:
        final_state = counter.most_common(1)[0][0]
        reason = "Majority voting"

    print("\n✅ FINAL RESULT")
    print(f"CSV status : {final_state}")
    print(f"Reason     : {reason}")

# ================= RUN =================
if __name__ == "__main__":
    main()
